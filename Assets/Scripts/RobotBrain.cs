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
    [SerializeField] private TrainingArena trainingArena;

    [Header("Reward Settings")]
    [SerializeField] private float timePenalty = -0.001f;
    [SerializeField] private float ballInSightReward = 0.005f;
    [Header("Gripper Assistance")]
    [SerializeField] private bool autoCloseGripperOnBallDetect = true;

    private Rigidbody body;
    private Vector3 startPosition;
    private Quaternion startRotation;
    private float lastKnownBallDirection = 0f;
    private float timeSinceLastDetection = 0f;
    private float previousDistanceToBall = float.MaxValue;

    public override void Initialize()
    {
        body = GetComponent<Rigidbody>();

        if (trackController == null) trackController = GetComponent<TrackController>();
        if (virtualSensors == null) virtualSensors = GetComponent<VirtualSensors>();
        if (yoloCamera == null) yoloCamera = GetComponentInChildren<SimulatedYoloCamera>();
        if (gripperController == null) gripperController = GetComponentInChildren<GripperController>();
        if (armRig == null) armRig = GetComponentInChildren<GfsxArmRigController>();
        if (sensorHeadRig == null) sensorHeadRig = GetComponentInChildren<GfsxSensorHeadRigController>();
        if (trainingArena == null) trainingArena = GetComponentInParent<TrainingArena>();

        startPosition = body != null ? body.position : transform.position;
        startRotation = body != null ? body.rotation : transform.rotation;

        if (trackController != null) trackController.KeyboardControlEnabled = false;
        if (sensorHeadRig != null) sensorHeadRig.KeyboardControlEnabled = false;
        if (armRig != null) armRig.KeyboardControlEnabled = false;
    }

    public override void OnEpisodeBegin()
    {
        if (trackController != null) trackController.Stop(immediate: true);

        if (gripperController != null && gripperController.IsHolding)
            gripperController.Release();

        if (sensorHeadRig != null) sensorHeadRig.ResetToNeutral();
        if (armRig != null) armRig.SetFloorPickupPose();

        if (virtualSensors != null)
            virtualSensors.RandomizeEpisodeNoise();
        if (yoloCamera != null)
            yoloCamera.RandomizeEpisodeNoise();

        if (trainingArena != null)
        {
            trainingArena.ResetEpisodeLayout();
        }
        else
        {
            TeleportToArenaPose(startPosition, startRotation);
        }

        Physics.SyncTransforms();

        lastKnownBallDirection = 0f;
        timeSinceLastDetection = 0f;
        previousDistanceToBall = float.MaxValue;
    }

    /// <summary>
    /// Used by TrainingArena to place the robot after layout randomization.
    /// </summary>
    public void TeleportToArenaPose(Vector3 worldPosition, Quaternion worldRotation)
    {
        if (trackController != null)
            trackController.Stop(immediate: true);

        if (body != null)
        {
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            body.position = worldPosition;
            body.rotation = worldRotation;
            body.WakeUp();
        }
        else
        {
            transform.SetPositionAndRotation(worldPosition, worldRotation);
        }

        startPosition = worldPosition;
        startRotation = worldRotation;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (yoloCamera != null)
        {
            yoloCamera.RefreshDetection();

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

        Vector3 currentPosition = body != null ? body.position : transform.position;
        Vector3 offsetFromStart = currentPosition - startPosition;

        Quaternion currentRotation = body != null ? body.rotation : transform.rotation;
        float headingDegrees = currentRotation.eulerAngles.y;
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
            trackController.SetCommand(moveCmd, turnCmd);

        if (sensorHeadRig != null)
        {
            float degreesPerSecond = 60f;
            float newPan = sensorHeadRig.S5PanAngle + (servoPanCmd * degreesPerSecond * Time.fixedDeltaTime);
            newPan = Mathf.Clamp(newPan, sensorHeadRig.S5MinimumAngle, sensorHeadRig.S5MaximumAngle);
            sensorHeadRig.SetPan(newPan);
        }

        ExecuteGripperAction(actions.DiscreteActions[0]);
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
            Vector3 robotPosition = body != null ? body.position : transform.position;
            float currentDistance = Vector3.Distance(robotPosition, yoloCamera.targetBall.position);

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
                AddReward(-0.01f);
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

        if (ShouldAutoCloseGripper())
        {
            armRig.SetJawClosed();
            return;
        }

        switch (command)
        {
            case 1:
                armRig.SetJawClosed();
                break;
            case 2:
                armRig.SetJawOpen();
                break;
        }
    }

    private bool ShouldAutoCloseGripper()
    {
        if (!autoCloseGripperOnBallDetect ||
            armRig == null ||
            virtualSensors == null ||
            gripperController == null)
        {
            return false;
        }

        if (gripperController.IsHolding || armRig.IsJawClosed)
            return false;

        return virtualSensors.GripperIrDetected;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        var discreteActions = actionsOut.DiscreteActions;
        continuousActions[0] = 0f;
        continuousActions[1] = 0f;
        continuousActions[2] = 0f;
        discreteActions[0] = 0;
    }
}
