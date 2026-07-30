using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// FixedArm Stage-2 relay: drive + pan camera (tilt locked like best catch runs),
/// fixed claw, seek → grasp (hold-lock) → return ball to red home cube.
/// Obs/action space matches fixed_arm_30m_relay_x4 for weight transfer.
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

    [Header("Reward Settings (catch-weighted)")]
    [SerializeField] private float timePenalty = -0.0008f;
    [SerializeField] private float ballInSightReward = 0.0004f;
    [SerializeField] private float approachRewardScale = 0.7f;
    [SerializeField] private float closeApproachBoost = 1.2f;
    [SerializeField, Min(0.1f)] private float ballZoneRadius = 0.45f;
    [SerializeField] private float ballZoneReward = 1.0f;
    [Tooltip("Must dominate dense shaping — otherwise the policy farms approach without grasping.")]
    [SerializeField] private float catchSuccessReward = 15.0f;
    [SerializeField] private float returnZoneReward = 3.0f;
    [SerializeField] private float returnWithBallReward = 5.0f;
    [SerializeField] private float collisionPenalty = 0.3f;
    [SerializeField, Min(0)] private int maxCollisionPenaltiesPerPhase = 5;
    [SerializeField, Min(0.05f)] private float homeZoneRadius = 0.4f;
    [SerializeField] private float returnApproachScale = 1.0f;
    [Tooltip("Centering help near the ball; keep small vs catch reward.")]
    [SerializeField] private float centerAlignReward = 0.003f;
    [Tooltip("Steps the ball must stay held before Return phase starts.")]
    [SerializeField, Min(1)] private int holdConfirmSteps = 10;
    [SerializeField] private float lingerInZoneWithoutGrabPenalty = -0.004f;
    [SerializeField] private float gripperIrAlignReward = 0.02f;
    [SerializeField] private float missCatchTimeoutPenalty = -4.0f;

    [Header("Gripper Assistance")]
    [SerializeField] private bool autoCloseGripperOnBallDetect = true;
    [Tooltip("While holding, ignore Open actions and keep jaws locked.")]
    [SerializeField] private bool lockJawClosedWhileHolding = true;

    [Header("Camera head (from best catch models: pan-only, fixed tilt)")]
    [Tooltip("Locked S6 tilt. Best catch runs used −15°.")]
    [SerializeField] private float episodeCameraTiltDegrees = -15f;
    [SerializeField] private float cameraPanDegreesPerSecond = 60f;
    [SerializeField, Range(0f, 0.4f)] private float cameraCommandDeadzone = 0.12f;

    [Header("Ball sight memory & camera search")]
    [SerializeField] private bool enableBallSearchAssist = true;
    [SerializeField, Range(1f, 2f)] private float searchCameraSpeedMultiplier = 1.45f;
    [SerializeField, Range(0f, 0.5f)] private float searchAssistStrength = 0.28f;
    [SerializeField, Min(0.5f)] private float ballMemoryDecaySeconds = 4f;
    [SerializeField] private float searchPanReward = 0.002f;

    [Header("Episode limits")]
    [SerializeField, Min(0)] private int maxEpisodeSteps = 5000;
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
    private int holdStreak;

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
        EnsureBehaviorSpace();
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
        holdStreak = 0;

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

        startPosition = body != null ? body.position : transform.position;
        startRotation = body != null ? body.rotation : transform.rotation;

        lastKnownBallDirection = 0f;
        timeSinceLastDetection = 0f;
        previousDistanceToBall = float.MaxValue;
        previousDistanceToHome = float.MaxValue;
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

    private void EnsureBehaviorSpace()
    {
        BehaviorParameters behavior = GetComponent<BehaviorParameters>();
        if (behavior == null)
            return;

        behavior.BrainParameters.VectorObservationSize = VectorObservationCount;
        behavior.BrainParameters.NumStackedVectorObservations = Mathf.Max(
            4,
            behavior.BrainParameters.NumStackedVectorObservations);
        behavior.BrainParameters.ActionSpec = new ActionSpec(ContinuousActionCount, new[] { 3 });
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

    private Vector3 GetHomeTarget()
    {
        if (trainingArena != null)
            return trainingArena.ReturnTargetWorld;
        return startPosition;
    }

    private float LockedTiltDegrees()
    {
        if (sensorHeadRig == null)
            return episodeCameraTiltDegrees;
        return Mathf.Clamp(
            episodeCameraTiltDegrees,
            sensorHeadRig.S6MinimumAngle,
            sensorHeadRig.S6MaximumAngle);
    }

    private float BallMemoryConfidence()
    {
        if (ballMemoryDecaySeconds <= 0.001f)
            return 0f;
        return 1f - Mathf.Clamp01(timeSinceLastDetection / ballMemoryDecaySeconds);
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

        // Match relay obs layout exactly (15): pan normalize by S5 max.
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
            if (!episodeEndedInCatch)
                AddReward(missCatchTimeoutPenalty);
            EndEpisode();
            return;
        }

        float moveCmd = actions.ContinuousActions[0];
        float turnCmd = actions.ContinuousActions[1];
        float servoPanCmd = ApplyDeadzone(actions.ContinuousActions[2], cameraCommandDeadzone);

        bool searching =
            enableBallSearchAssist &&
            yoloCamera != null &&
            !yoloCamera.IsBallVisible &&
            BallMemoryConfidence() > 0.05f &&
            Mathf.Abs(lastKnownBallDirection) > 0.01f;

        if (searching)
        {
            servoPanCmd = Mathf.Clamp(
                servoPanCmd + searchAssistStrength * lastKnownBallDirection,
                -1f,
                1f);
            if (Mathf.Sign(servoPanCmd) == Mathf.Sign(lastKnownBallDirection) &&
                Mathf.Abs(servoPanCmd) > 0.05f)
            {
                AddReward(searchPanReward * BallMemoryConfidence());
            }
        }

        if (trackController != null)
            trackController.SetCommand(moveCmd, turnCmd);

        if (sensorHeadRig != null)
        {
            float panSpeed = cameraPanDegreesPerSecond;
            if (searching)
                panSpeed *= searchCameraSpeedMultiplier;

            float newPan = sensorHeadRig.S5PanAngle +
                           (servoPanCmd * panSpeed * Time.fixedDeltaTime);
            newPan = Mathf.Clamp(newPan, sensorHeadRig.S5MinimumAngle, sensorHeadRig.S5MaximumAngle);
            sensorHeadRig.SetServoCommands(newPan, LockedTiltDegrees());
        }

        if (armRig != null)
            armRig.SetFloorPickupPoseKeepingJaw();

        ExecuteGripperAction(actions.DiscreteActions.Length > 0 ? actions.DiscreteActions[0] : 0);
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
        bool holding = gripperController != null && gripperController.IsHolding;

        if (!holding && yoloCamera != null && yoloCamera.IsBallVisible)
        {
            float proximity = 1f - Mathf.Clamp01(yoloCamera.NormalizedDistance);
            AddReward(ballInSightReward * (0.2f + 0.8f * proximity));

            float center = 1f - Mathf.Clamp01(Mathf.Abs(yoloCamera.RelativeAngleX));
            if (proximity > 0.35f)
                AddReward(centerAlignReward * center * proximity);
        }

        if (!holding && distanceToBall < float.MaxValue)
        {
            if (previousDistanceToBall < float.MaxValue)
            {
                float distanceDelta = previousDistanceToBall - distanceToBall;
                float closeness = 1f - Mathf.Clamp01(distanceToBall / 2.5f);
                float scale = approachRewardScale * (1f + closeApproachBoost * closeness);
                AddReward(Mathf.Clamp(distanceDelta * scale, -0.05f, 0.08f));
            }

            previousDistanceToBall = distanceToBall;

            if (!awardedBallZone && distanceToBall <= ballZoneRadius)
            {
                awardedBallZone = true;
                float zonePay = Mathf.Max(0f, ballZoneReward - seekCollisions * collisionPenalty);
                AddReward(zonePay);
            }

            if (awardedBallZone && distanceToBall <= ballZoneRadius)
                AddReward(lingerInZoneWithoutGrabPenalty);
        }

        if (!holding &&
            virtualSensors != null &&
            virtualSensors.GripperIrDetected &&
            distanceToBall < 0.35f)
        {
            AddReward(gripperIrAlignReward);
        }

        if (holding)
        {
            holdStreak++;
            episodeEndedInCatch = true;
            if (holdStreak >= holdConfirmSteps)
            {
                AddReward(catchSuccessReward);
                phase = EpisodePhase.Return;
                previousDistanceToHome = Vector3.Distance(robotPosition, GetHomeTarget());
                returnCollisions = 0;
                obstacleLatched = false;
            }
            else
            {
                AddReward(0.05f);
            }
        }
        else
        {
            if (holdStreak > 0)
                AddReward(-0.5f);
            holdStreak = 0;
        }
    }

    private void EvaluateReturnRewards(Vector3 robotPosition)
    {
        if (gripperController == null || !gripperController.IsHolding)
        {
            AddReward(-0.6f);
            phase = EpisodePhase.Seek;
            holdStreak = 0;
            previousDistanceToBall = float.MaxValue;
            return;
        }

        AddReward(0.005f);

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
        if (armRig == null)
            return;

        bool holding = gripperController != null && gripperController.IsHolding;
        if (holding && lockJawClosedWhileHolding)
        {
            armRig.SetJawClosed();
            return;
        }

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

    private static float ApplyDeadzone(float value, float deadzone)
    {
        float mag = Mathf.Abs(value);
        if (mag <= deadzone)
            return 0f;
        float sign = Mathf.Sign(value);
        return sign * ((mag - deadzone) / Mathf.Max(0.001f, 1f - deadzone));
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        var discreteActions = actionsOut.DiscreteActions;
        for (int index = 0; index < continuousActions.Length; index++)
            continuousActions[index] = 0f;
        if (discreteActions.Length > 0)
            discreteActions[0] = 0;
    }
}
