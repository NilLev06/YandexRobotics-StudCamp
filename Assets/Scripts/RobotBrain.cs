using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class RobotBrain : Agent
{
    [Header("Robot Components")]
    [SerializeField] private TrackController trackController;
    [SerializeField] private VirtualSensors virtualSensors;
    [SerializeField] private SimulatedYoloCamera yoloCamera;
    [SerializeField] private GripperController gripperController;
    [SerializeField] private GfsxArmRigController armRig;
    [SerializeField] private GfsxSensorHeadRigController sensorHeadRig;

    [Header("Reward Settings")]
    [SerializeField] private float timePenalty = -0.001f;
    [SerializeField] private float ballInSightReward = 0.005f;

    private Vector3 startPosition;
    private float lastKnownBallDirection = 0f;
    private float timeSinceLastDetection = 0f;
    private float previousDistanceToBall = float.MaxValue;

    public override void Initialize()
    {
        if (trackController == null) trackController = GetComponent<TrackController>();
        if (virtualSensors == null) virtualSensors = GetComponent<VirtualSensors>();
        if (gripperController == null) gripperController = GetComponentInParent<GripperController>();
        if (armRig == null) armRig = GetComponentInChildren<GfsxArmRigController>();
        if (sensorHeadRig == null) sensorHeadRig = GetComponentInChildren<GfsxSensorHeadRigController>();

        startPosition = transform.position;

        if (trackController != null) trackController.KeyboardControlEnabled = false;
        if (sensorHeadRig != null) sensorHeadRig.KeyboardControlEnabled = false;
        if (armRig != null) SetPrivateField(armRig, "readKeyboard", false);
    }

    public override void OnEpisodeBegin()
    {
        transform.position = startPosition;
        transform.rotation = Quaternion.identity;

        if (trackController != null) trackController.Stop(immediate: true);

        if (gripperController != null && gripperController.IsHolding)
        {
            gripperController.Release();
        }

        if (sensorHeadRig != null)
        {
            SetPrivateField(sensorHeadRig, "s5PanAngle", 0f);
            SetPrivateField(sensorHeadRig, "s6TiltAngle", 0f);
        }

        if (armRig != null)
        {
            SetPrivateField(armRig, "s4Closure", armRig.S4OpenPositionDegrees);
        }

        lastKnownBallDirection = 0f;
        timeSinceLastDetection = 0f;
        previousDistanceToBall = float.MaxValue;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (yoloCamera != null)
        {
            if (yoloCamera.IsBallVisible)
            {
                timeSinceLastDetection = 0f;
                lastKnownBallDirection = Mathf.Sign(yoloCamera.RelativeAngleX);
            }
            else
            {
                timeSinceLastDetection += Time.fixedDeltaTime;
            }
        }

        Vector3 offsetFromStart = transform.position - startPosition;

        float headingDegrees = transform.eulerAngles.y;
        if (headingDegrees > 180f) headingDegrees -= 360f;
        float normalizedHeading = headingDegrees / 180f;

        float currentSpeed = trackController != null ? trackController.LinearSpeed : 0f;
        bool isHoldingBall = gripperController != null && gripperController.IsHolding;

        float currentServoNormalize = 0f;
        if (sensorHeadRig != null && sensorHeadRig.S5MaximumAngle > 0.001f)
        {
            currentServoNormalize = sensorHeadRig.S5PanAngle / sensorHeadRig.S5MaximumAngle;
        }

        sensor.AddObservation(virtualSensors != null ? virtualSensors.UltrasonicNormalized : 1f);
        sensor.AddObservation(virtualSensors != null ? virtualSensors.LeftIr : 0f);
        sensor.AddObservation(virtualSensors != null ? virtualSensors.RightIr : 0f);
        sensor.AddObservation(virtualSensors != null ? virtualSensors.GripperIr : 0f);
        sensor.AddObservation(yoloCamera != null && yoloCamera.IsBallVisible ? yoloCamera.RelativeAngleX : 0f);
        sensor.AddObservation(yoloCamera != null && yoloCamera.IsBallVisible ? yoloCamera.NormalizedDistance : 1f);
        sensor.AddObservation(lastKnownBallDirection);
        sensor.AddObservation(yoloCamera != null && yoloCamera.IsBallVisible ? 1.0f : 0.0f);
        sensor.AddObservation(currentServoNormalize);
        sensor.AddObservation(isHoldingBall ? 1.0f : 0.0f);
        sensor.AddObservation(offsetFromStart.x);
        sensor.AddObservation(offsetFromStart.z);
        sensor.AddObservation(normalizedHeading);
        sensor.AddObservation(currentSpeed);
        sensor.AddObservation(timeSinceLastDetection);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveCmd = actions.ContinuousActions[0];
        float turnCmd = actions.ContinuousActions[1];
        float servoPanCmd = actions.ContinuousActions[2];

        if (trackController != null)
        {
            trackController.SetCommand(moveCmd, turnCmd);
        }

        if (sensorHeadRig != null)
        {
            float degreesPerSecond = 60f;
            float newPan = sensorHeadRig.S5PanAngle + (servoPanCmd * degreesPerSecond * Time.fixedDeltaTime);
            newPan = Mathf.Clamp(newPan, sensorHeadRig.S5MinimumAngle, sensorHeadRig.S5MaximumAngle);
            SetPrivateField(sensorHeadRig, "s5PanAngle", newPan);
        }

        int gripperCmd = actions.DiscreteActions[0];
        ExecuteGripperAction(gripperCmd);

        EvaluateRewards();
    }

    private void EvaluateRewards()
    {
        AddReward(timePenalty);

        if (yoloCamera != null && yoloCamera.IsBallVisible)
        {
            AddReward(ballInSightReward);
            float centerBonus = 0.005f * (1.0f - Mathf.Abs(yoloCamera.RelativeAngleX));
            AddReward(centerBonus);
        }

        if (yoloCamera != null && yoloCamera.targetBall != null)
        {
            float currentDistance = Vector3.Distance(transform.position, yoloCamera.targetBall.position);

            if (previousDistanceToBall < float.MaxValue)
            {
                float distanceDelta = previousDistanceToBall - currentDistance;
                AddReward(distanceDelta * 1.0f);
            }
            previousDistanceToBall = currentDistance;
        }

        if (virtualSensors != null)
        {
            if (virtualSensors.LeftIrDetected || virtualSensors.RightIrDetected || virtualSensors.UltrasonicNormalized < 0.1f)
            {
                AddReward(-0.01f);
            }
        }

        if (gripperController != null && gripperController.IsHolding)
        {
            SetReward(2.0f);
            EndEpisode();
        }
    }

    private void ExecuteGripperAction(int command)
    {
        if (armRig == null) return;

        switch (command)
        {
            case 0:
                break;
            case 1:
                SetPrivateField(armRig, "s4Closure", armRig.S4ClosedPositionDegrees);
                break;
            case 2:
                SetPrivateField(armRig, "s4Closure", armRig.S4OpenPositionDegrees);
                break;
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        var discreteActions = actionsOut.DiscreteActions;

        continuousActions[0] = Input.GetAxisRaw("Vertical");
        continuousActions[1] = Input.GetAxisRaw("Horizontal");

        if (Input.GetKey(KeyCode.Q)) continuousActions[2] = -1f;
        else if (Input.GetKey(KeyCode.E)) continuousActions[2] = 1f;
        else continuousActions[2] = 0f;

        if (Input.GetKey(KeyCode.Alpha1)) discreteActions[0] = 1;
        else if (Input.GetKey(KeyCode.Alpha2)) discreteActions[0] = 2;
        else discreteActions[0] = 0;
    }

    private void SetPrivateField(object target, string fieldName, object value)
    {
        var field = target.GetType().GetField(fieldName, System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        if (field != null) field.SetValue(target, value);
    }
}
