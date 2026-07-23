using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// FixedArm agent for StudCamp Stage 2 (relay):
/// drive + camera pan, fixed claw, approach ball zone → grasp → return to start.
/// Rewards scaled to the competition scoring table.
/// </summary>
public class RobotBrain : Agent
{
    private const int ContinuousActionCount = 3;
    private const int VectorObservationCount = 15;

    private enum EpisodePhase
    {
        Seek,
        Return
    }

    [Header("Training mode")]
    [SerializeField] private RobotTrainingMode trainingMode = RobotTrainingMode.FixedArm;

    [Header("Robot Components")]
    [SerializeField] private TrackController trackController;
    [SerializeField] private VirtualSensors virtualSensors;
    [SerializeField] private SimulatedYoloCamera yoloCamera;
    [SerializeField] private GripperController gripperController;
    [SerializeField] private GfsxArmRigController armRig;
    [SerializeField] private GfsxSensorHeadRigController sensorHeadRig;
    [SerializeField] private TrainingArena trainingArena;

    [Header("Reward Settings (Stage 2 scaled)")]
    [SerializeField] private float timePenalty = -0.0005f;
    [Tooltip("Small look bonus; keep low so agent does not farm sight from afar.")]
    [SerializeField] private float ballInSightReward = 0.001f;
    [Tooltip("Base multiplier for closing distance to the ball.")]
    [SerializeField] private float approachRewardScale = 2.0f;
    [Tooltip("Extra approach boost when already close (anti fear of final metres).")]
    [SerializeField] private float closeApproachBoost = 3.0f;
    [Tooltip("Distance to ball that counts as ball-zone entry (tape zone).")]
    [SerializeField, Min(0.1f)] private float ballZoneRadius = 0.45f;
    [Tooltip("Maps to +15 for entering the ball zone.")]
    [SerializeField] private float ballZoneReward = 1.5f;
    [Tooltip("Maps to +20 for grasp.")]
    [SerializeField] private float catchSuccessReward = 2.0f;
    [Tooltip("Maps to +15 for returning to the red home cube.")]
    [SerializeField] private float returnZoneReward = 1.5f;
    [Tooltip("Maps to +15 for finishing while still holding the ball.")]
    [SerializeField] private float returnWithBallReward = 1.5f;
    [Tooltip("Maps to -3 collision; deducted from zone bonuses, floored at 0.")]
    [SerializeField] private float collisionPenalty = 0.3f;
    [SerializeField, Min(0)] private int maxCollisionPenaltiesPerPhase = 5;
    [SerializeField, Min(0.05f)] private float homeZoneRadius = 0.55f;
    [SerializeField] private float returnApproachScale = 1.5f;

    [Header("Gripper Assistance")]
    [SerializeField] private bool autoCloseGripperOnBallDetect = true;

    [Header("Camera head")]
    [Tooltip("Episode start tilt (S6). Agent controls pan (S5) only.")]
    [SerializeField] private float episodeCameraTiltDegrees = -15f;

    [Header("Episode limits")]
    [SerializeField, Min(0)] private int maxEpisodeSteps = 6000;
    [SerializeField] private float episodeTimeoutPenalty = -1.0f;

    private Rigidbody body;
    private Vector3 startPosition;
    private Quaternion startRotation;
    private float lastKnownBallDirection;
    private float timeSinceLastDetection;
    private float previousDistanceToBall = float.MaxValue;
    private float previousDistanceToHome = float.MaxValue;
    private bool episodeLifecycleStarted;
    private bool episodeEndedInCatch;
    private EpisodePhase phase;
    private bool awardedBallZone;
    private bool awardedReturnZone;
    private int seekCollisions;
    private int returnCollisions;
    private bool obstacleLatched;

    public RobotTrainingMode TrainingMode => trainingMode;
    public int ExpectedContinuousActions => ContinuousActionCount;
    public int ExpectedVectorObservations => VectorObservationCount;

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

        if (trackController != null)
            trackController.CaptureDriveBaseline();

        if (sensorHeadRig != null) sensorHeadRig.KeyboardControlEnabled = false;
        if (armRig != null) armRig.KeyboardControlEnabled = false;

        trainingMode = RobotTrainingMode.FixedArm;
        EnsureTrainingBehaviorWhenCommunicatorOn();
    }

    public override void OnEpisodeBegin()
    {
        if (episodeLifecycleStarted)
            TrainingGraspMetrics.RegisterEpisodeEnd(episodeEndedInCatch);
        else
            episodeLifecycleStarted = true;

        episodeEndedInCatch = false;
        phase = EpisodePhase.Seek;
        awardedBallZone = false;
        awardedReturnZone = false;
        seekCollisions = 0;
        returnCollisions = 0;
        obstacleLatched = false;

        if (trackController != null) trackController.Stop(immediate: true);

        if (gripperController != null && gripperController.IsHolding)
            gripperController.Release();

        if (sensorHeadRig != null)
            ResetCameraHeadForEpisode();

        if (armRig != null)
            armRig.SetFloorPickupPose();

        if (virtualSensors != null)
            virtualSensors.RandomizeEpisodeNoise();
        if (yoloCamera != null)
            yoloCamera.RandomizeEpisodeNoise();

        if (trainingArena != null)
            trainingArena.ResetEpisodeLayout();
        else
            TeleportToArenaPose(startPosition, startRotation);

        Physics.SyncTransforms();

        // Re-capture start after arena reset (spawn may have moved).
        startPosition = body != null ? body.position : transform.position;
        startRotation = body != null ? body.rotation : transform.rotation;

        lastKnownBallDirection = 0f;
        timeSinceLastDetection = 0f;
        previousDistanceToBall = float.MaxValue;
        previousDistanceToHome = float.MaxValue;
    }

    private Vector3 GetHomeTarget()
    {
        if (trainingArena != null)
            return trainingArena.ReturnTargetWorld;
        return startPosition;
    }

    private void ResetCameraHeadForEpisode()
    {
        sensorHeadRig.SetPan(0f);
        float tilt = Mathf.Clamp(
            episodeCameraTiltDegrees,
            sensorHeadRig.S6MinimumAngle,
            sensorHeadRig.S6MaximumAngle);
        sensorHeadRig.SetTilt(tilt);
    }

    public void BindTrainingArena(TrainingArena arena)
    {
        trainingArena = arena;
    }

    private void EnsureTrainingBehaviorWhenCommunicatorOn()
    {
        if (!Academy.IsInitialized || !Academy.Instance.IsCommunicatorOn)
            return;

        BehaviorParameters behavior = GetComponent<BehaviorParameters>();
        if (behavior == null)
            return;

        if (behavior.BehaviorType == BehaviorType.InferenceOnly)
        {
            behavior.BehaviorType = BehaviorType.Default;
            behavior.Model = null;
            Debug.LogWarning(
                $"RobotBrain on {name}: BehaviorType was InferenceOnly while training — forced Default.");
        }
    }

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
                lastKnownBallDirection = Mathf.Abs(yoloCamera.RelativeAngleX) < 0.05f
                    ? 0f
                    : Mathf.Sign(yoloCamera.RelativeAngleX);
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
            currentServoNormalize = sensorHeadRig.S5PanAngle / sensorHeadRig.S5MaximumAngle;

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
        if (maxEpisodeSteps > 0 && StepCount >= maxEpisodeSteps)
        {
            AddReward(episodeTimeoutPenalty);
            EndEpisode();
            return;
        }

        float moveCmd = actions.ContinuousActions[0];
        float turnCmd = actions.ContinuousActions[1];
        float servoPanCmd = actions.ContinuousActions[2];

        if (trackController != null)
            trackController.SetCommand(moveCmd, turnCmd);

        if (sensorHeadRig != null)
        {
            const float degreesPerSecond = 60f;
            float newPan = sensorHeadRig.S5PanAngle +
                           (servoPanCmd * degreesPerSecond * Time.fixedDeltaTime);
            newPan = Mathf.Clamp(
                newPan,
                sensorHeadRig.S5MinimumAngle,
                sensorHeadRig.S5MaximumAngle);
            sensorHeadRig.SetServoCommands(
                newPan,
                Mathf.Clamp(
                    episodeCameraTiltDegrees,
                    sensorHeadRig.S6MinimumAngle,
                    sensorHeadRig.S6MaximumAngle));
        }

        if (armRig != null)
            armRig.SetFloorPickupPoseKeepingJaw();

        ExecuteGripperAction(actions.DiscreteActions[0]);
        EvaluateRewards();
    }

    private void EvaluateRewards()
    {
        AddReward(timePenalty);
        ApplyCollisionPenalty();

        Vector3 robotPosition = body != null ? body.position : transform.position;
        Transform ball = yoloCamera != null ? yoloCamera.targetBall : null;
        float distanceToBall = ball != null
            ? Vector3.Distance(robotPosition, ball.position)
            : float.MaxValue;

        if (phase == EpisodePhase.Seek)
            EvaluateSeekRewards(robotPosition, distanceToBall);
        else
            EvaluateReturnRewards(robotPosition);
    }

    private void EvaluateSeekRewards(Vector3 robotPosition, float distanceToBall)
    {
        if (yoloCamera != null && yoloCamera.IsBallVisible)
        {
            // Prefer closing distance over staring from afar.
            float proximity = 1f - Mathf.Clamp01(yoloCamera.NormalizedDistance);
            AddReward(ballInSightReward * (0.25f + 0.75f * proximity));
            AddReward(0.004f * (1.0f - Mathf.Abs(yoloCamera.RelativeAngleX)) * proximity);
        }

        if (distanceToBall < float.MaxValue)
        {
            if (previousDistanceToBall < float.MaxValue)
            {
                float distanceDelta = previousDistanceToBall - distanceToBall;
                float closeness = 1f - Mathf.Clamp01(distanceToBall / 2.5f);
                float scale = approachRewardScale * (1f + closeApproachBoost * closeness * closeness);
                AddReward(distanceDelta * scale);
            }

            previousDistanceToBall = distanceToBall;

            if (!awardedBallZone && distanceToBall <= ballZoneRadius)
            {
                awardedBallZone = true;
                float zonePay = Mathf.Max(0f, ballZoneReward - seekCollisions * collisionPenalty);
                AddReward(zonePay);
            }
        }

        if (gripperController != null && gripperController.IsHolding)
        {
            episodeEndedInCatch = true;
            AddReward(catchSuccessReward);
            phase = EpisodePhase.Return;
            previousDistanceToHome = Vector3.Distance(robotPosition, GetHomeTarget());
            returnCollisions = 0;
            obstacleLatched = false;
        }
    }

    private void EvaluateReturnRewards(Vector3 robotPosition)
    {
        // Dropped the ball during return — soft fail, keep seeking again if still near.
        if (gripperController == null || !gripperController.IsHolding)
        {
            AddReward(-0.5f);
            phase = EpisodePhase.Seek;
            previousDistanceToBall = float.MaxValue;
            return;
        }

        Vector3 home = GetHomeTarget();
        float distanceToHome = Vector3.Distance(robotPosition, home);
        if (previousDistanceToHome < float.MaxValue)
        {
            float delta = previousDistanceToHome - distanceToHome;
            AddReward(delta * returnApproachScale);
        }

        previousDistanceToHome = distanceToHome;

        if (!awardedReturnZone && distanceToHome <= homeZoneRadius)
        {
            awardedReturnZone = true;
            float zonePay = Mathf.Max(0f, returnZoneReward - returnCollisions * collisionPenalty);
            AddReward(zonePay);
            AddReward(returnWithBallReward);
            EndEpisode();
        }
    }

    private void ApplyCollisionPenalty()
    {
        if (virtualSensors == null)
            return;

        // Near the ball / gripper IR: sensors often see the ball itself — do not scare the agent away.
        bool nearBall = false;
        if (yoloCamera != null && yoloCamera.targetBall != null)
        {
            Vector3 robotPosition = body != null ? body.position : transform.position;
            nearBall = Vector3.Distance(robotPosition, yoloCamera.targetBall.position) < 0.55f;
        }

        if (nearBall || (virtualSensors.GripperIrDetected && phase == EpisodePhase.Seek))
        {
            obstacleLatched = false;
            return;
        }

        bool hit =
            virtualSensors.LeftIrDetected ||
            virtualSensors.RightIrDetected ||
            virtualSensors.UltrasonicNormalized < 0.12f;

        if (!hit)
        {
            obstacleLatched = false;
            return;
        }

        if (obstacleLatched)
            return;

        obstacleLatched = true;
        if (phase == EpisodePhase.Seek)
        {
            if (seekCollisions < maxCollisionPenaltiesPerPhase)
            {
                seekCollisions++;
                // Soft step cost; zone bonus already floors collision deductions.
                AddReward(-0.02f);
            }
        }
        else if (returnCollisions < maxCollisionPenaltiesPerPhase)
        {
            returnCollisions++;
            AddReward(-0.02f);
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
        for (int index = 0; index < continuousActions.Length; index++)
            continuousActions[index] = 0f;
        discreteActions[0] = 0;
    }
}
