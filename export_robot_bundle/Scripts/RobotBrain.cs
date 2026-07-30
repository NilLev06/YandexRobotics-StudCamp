using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class RobotBrain : Agent
{
    private const int DriveContinuousActions = 3;
    private const int MobileArmContinuousActions = 5;

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

    [Header("Domain randomization")]
    [SerializeField] private bool enableDomainRandomization = true;
    [SerializeField] private Vector2 robotMassRange = new Vector2(1.0f, 4.0f);
    [SerializeField, Range(0, 20)] private int actionLatencyMinSteps = 8;
    [SerializeField, Range(0, 20)] private int actionLatencyMaxSteps = 13;

    [Header("Reward Settings")]
    [SerializeField] private float timePenalty = -0.0012f;
    [SerializeField] private float ballInSightReward = 0.0015f;
    [SerializeField] private float gripperIrReward = 0.02f;
    [SerializeField] private float emptyGripperClosePenalty = -0.35f;
    [SerializeField] private float armOcclusionPenalty = -0.008f;
    [SerializeField] private float holdStepReward = 0.02f;
    [SerializeField] private float holdSuccessReward = 15.0f;
    [SerializeField] private int holdSuccessSteps = 40;
    [SerializeField] private float bodyTurnPenalty = -0.01f;
    [SerializeField] private float cameraPanReward = 0.008f;
    [SerializeField] private float bodyTurnWhileBallOffCenterPenalty = -0.02f;
    [Tooltip("Extra penalty when the chassis yaws quickly with almost no forward speed.")]
    [SerializeField] private float spinInPlacePenalty = -0.03f;
    [SerializeField, Min(5f)] private float spinInPlaceYawThreshold = 22f;
    [SerializeField, Min(0.01f)] private float spinInPlaceSpeedThreshold = 0.05f;
    [Tooltip("Penalty when the ball is already centred but the body still turns.")]
    [SerializeField] private float bodyTurnWhileBallCenteredPenalty = -0.018f;
    [SerializeField] private float farArmMotionPenalty = -0.003f;
    [SerializeField] private float closeArmMotionReward = 0.003f;
    [SerializeField] private float rushNearBallPenalty = -0.006f;
    [SerializeField] private float ballKickPenalty = -0.1f;
    [SerializeField] private float slowApproachReward = 0.004f;
    [SerializeField] private float closeApproachDistance = 0.65f;
    [SerializeField] private float distanceShapingScale = 0.22f;
    [SerializeField] private float obstacleProximityPenalty = -0.012f;
    [SerializeField] private float armWallContactPenalty = -0.025f;
    [SerializeField] private float armFloorContactPenalty = -0.08f;
    [SerializeField] private float reverseDrivePenalty = -0.012f;
    [SerializeField] private float reverseWhileBallVisiblePenalty = -0.02f;
    [SerializeField] private float episodeTimeoutPenalty = -2.0f;

    [Header("Arm engagement (MobileArm)")]
    [Tooltip("Arm commands are scaled down to zero beyond this distance (metres).")]
    [SerializeField] private float armEngageFarDistance = 1.8f;
    [Tooltip("Arm commands reach full strength inside this distance (metres).")]
    [SerializeField] private float armEngageNearDistance = 0.85f;

    [Header("MobileArm pose penalties (ignored in FixedArm)")]
    [Tooltip("Hard S1 ceiling while the agent controls the arm (degrees). Prevents sky poses.")]
    [SerializeField] private float mobileTrainingS1Maximum = 0f;
    [Tooltip("Hard S1 floor while the agent controls the arm (degrees). Prevents digging into the ground.")]
    [SerializeField] private float mobileTrainingS1Minimum = -20f;
    [Tooltip("Hard S2 floor (most negative elbow). Stops the claw folding into the ground.")]
    [SerializeField] private float mobileTrainingS2Minimum = -35f;
    [Tooltip("Penalty scale when S1 is raised above the floor-pickup shoulder angle.")]
    [SerializeField] private float highArmPenalty = -0.015f;
    [Tooltip("S1 degrees above floor-pickup before high-arm penalty starts.")]
    [SerializeField] private float highArmS1SlackDegrees = 8f;
    [Tooltip("Penalty when HoldPoint / claw faces upward (sky).")]
    [SerializeField] private float skyClawPitchPenalty = -0.012f;
    [Tooltip("HoldPoint.forward.y above this counts as pointing up.")]
    [SerializeField, Range(0.05f, 0.9f)] private float skyClawPitchThreshold = 0.25f;
    [Tooltip("Soft reward for staying near the floor-pickup S1/S2 while far from the ball.")]
    [SerializeField] private float floorPoseBiasReward = 0.001f;
    [SerializeField] private float highHoldPointPenalty = -0.012f;
    [SerializeField] private float highHoldPointHeightMetres = 0.14f;
    [Tooltip("Penalty when S1 dives below the floor-pickup pose (claw scrapes ground).")]
    [SerializeField] private float lowArmDivePenalty = -0.025f;

    [Header("Gripper Assistance")]
    [SerializeField] private bool autoCloseGripperOnBallDetect = true;

    [Header("Ball sight memory & camera search")]
    [SerializeField] private bool enableBallSearchAssist = true;
    [Tooltip("Extra camera pan speed while the ball is off-screen but still remembered.")]
    [SerializeField, Range(1f, 2f)] private float searchCameraSpeedMultiplier = 1.65f;
    [Tooltip("Small assist added to pan command toward the last seen ball direction.")]
    [SerializeField, Range(0f, 0.5f)] private float searchAssistStrength = 0.28f;
    [Tooltip("How long the last sighting stays useful before fading out (seconds).")]
    [SerializeField, Min(0.5f)] private float ballMemoryDecaySeconds = 5f;
    [SerializeField] private float searchPanReward = 0.01f;

    [Header("Camera head (S5 pan = agent; S6 tilt locked)")]
    [Tooltip("S6 tilt is not an agent action. Locked to this angle each episode.")]
    [SerializeField] private float episodeCameraTiltDegrees = -15f;
    [SerializeField] private bool lockCameraTiltDuringTraining = true;
    [SerializeField] private float cameraPanAtLimitPenalty = -0.01f;
    [SerializeField] private float cameraPanOscillationPenalty = -0.015f;

    [Header("Camera coverage map")]
    [SerializeField] private bool enableCoverageRewards = true;
    [SerializeField] private float coverageRevealReward = 0.003f;

    [Header("Episode limits")]
    [Tooltip("0 = no step limit. Longer episodes give more time to search/approach.")]
    [SerializeField, Min(0)] private int maxEpisodeSteps = 4000;

    private Rigidbody body;
    private Vector3 startPosition;
    private Quaternion startRotation;
    private float lastKnownBallDirection;
    private float lastKnownBallAngle;
    private float lastKnownBallDistance = 1f;
    private float ballMemoryConfidence;
    private bool hasBallSightMemory;
    private float timeSinceLastDetection;
    private float previousDistanceToBall = float.MaxValue;
    private float previousHoldPointDistance = float.MaxValue;
    private float previousBallSpeed = float.MaxValue;
    private int holdTicks;
    private int burstDropoutRemaining;
    private int currentActionLatency;
    private readonly Queue<float[]> actionBuffer = new Queue<float[]>();
    private float lastTurnCommand;
    private float lastServoPanCommand;
    private float previousServoPanCommand;
    private float previousS1Angle;
    private float previousS2Angle;
    private ArmContactTracker armContactTracker;
    private CameraCoverageMap coverageMap;
    private bool episodeLifecycleStarted;
    private bool episodeEndedInCatch;

    public RobotTrainingMode TrainingMode => trainingMode;
    public int ExpectedContinuousActions =>
        trainingMode == RobotTrainingMode.MobileArm
            ? MobileArmContinuousActions
            : DriveContinuousActions;

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
        EnsureArmContactTracker();
        EnsureCoverageMap();
        EnsureTrainingBehaviorWhenCommunicatorOn();

        startPosition = body != null ? body.position : transform.position;
        startRotation = body != null ? body.rotation : transform.rotation;

        if (trackController != null)
        {
            trackController.CaptureDriveBaseline();
            trackController.KeyboardControlEnabled = false;
        }

        if (sensorHeadRig != null) sensorHeadRig.KeyboardControlEnabled = false;
        if (armRig != null) armRig.KeyboardControlEnabled = false;
    }

    public override void OnEpisodeBegin()
    {
        if (episodeLifecycleStarted)
            TrainingGraspMetrics.RegisterEpisodeEnd(episodeEndedInCatch);
        else
            episodeLifecycleStarted = true;

        episodeEndedInCatch = false;

        if (trackController != null) trackController.Stop(immediate: true);

        if (gripperController != null && gripperController.IsHolding)
            gripperController.Release();

        if (sensorHeadRig != null)
            ResetCameraHeadForEpisode();
        ResetArmPose();

        if (virtualSensors != null)
            virtualSensors.RandomizeEpisodeNoise();
        if (yoloCamera != null)
            yoloCamera.RandomizeEpisodeNoise();

        ApplyPhysicsDomainRandomization();
        ResetActionLatencyBuffer();

        if (trainingArena != null)
        {
            trainingArena.ResetEpisodeLayout();
        }
        else
        {
            TeleportToArenaPose(startPosition, startRotation);
        }

        Physics.SyncTransforms();

        if (coverageMap != null)
            coverageMap.ResetCoverage();

        lastKnownBallDirection = 0f;
        lastKnownBallAngle = 0f;
        lastKnownBallDistance = 1f;
        ballMemoryConfidence = 0f;
        hasBallSightMemory = false;
        timeSinceLastDetection = 0f;
        previousDistanceToBall = float.MaxValue;
        previousHoldPointDistance = float.MaxValue;
        previousBallSpeed = float.MaxValue;
        holdTicks = 0;
        burstDropoutRemaining = 0;
        lastTurnCommand = 0f;
        lastServoPanCommand = 0f;
        previousServoPanCommand = 0f;
        CacheArmAngleBaselines();
    }

    private void ResetCameraHeadForEpisode()
    {
        sensorHeadRig.SetPan(0f);
        float tilt = lockCameraTiltDuringTraining
            ? episodeCameraTiltDegrees
            : 0f;
        sensorHeadRig.SetTilt(
            Mathf.Clamp(
                tilt,
                sensorHeadRig.S6MinimumAngle,
                sensorHeadRig.S6MaximumAngle));
    }

    private void CacheArmAngleBaselines()
    {
        if (armRig == null)
            return;

        previousS1Angle = armRig.S1Angle;
        previousS2Angle = armRig.S2Angle;
    }

    private void ResetArmPose()
    {
        if (armRig == null)
            return;

        if (trainingMode == RobotTrainingMode.FixedArm)
            armRig.SetFloorPickupPose();
        else
            armRig.SetFloorPickupPose();
    }

    private void ApplyPhysicsDomainRandomization()
    {
        if (!ShouldApplyDomainRandomization())
            return;

        if (body != null)
            body.mass = Random.Range(robotMassRange.x, robotMassRange.y);

        if (trackController != null)
            trackController.ApplyTrainingRandomization();
    }

    private void ResetActionLatencyBuffer()
    {
        actionBuffer.Clear();
        currentActionLatency = 0;

        if (!ShouldApplyDomainRandomization())
            return;

        currentActionLatency = Random.Range(actionLatencyMinSteps, actionLatencyMaxSteps + 1);
        int actionWidth = ExpectedContinuousActions;
        for (int index = 0; index < currentActionLatency; index++)
        {
            actionBuffer.Enqueue(new float[actionWidth]);
        }
    }

    private bool ShouldApplyDomainRandomization()
    {
        return enableDomainRandomization && Academy.Instance.IsCommunicatorOn;
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

        // Visual Validation can leave InferenceOnly in the scene/build; training needs Default.
        if (behavior.BehaviorType == BehaviorType.InferenceOnly)
        {
            behavior.BehaviorType = BehaviorType.Default;
            behavior.Model = null;
            Debug.LogWarning(
                $"RobotBrain on {name}: BehaviorType was InferenceOnly while training — forced Default.");
        }
    }

    private void EnsureCoverageMap()
    {
        coverageMap = GetComponent<CameraCoverageMap>();
        if (coverageMap == null)
            coverageMap = gameObject.AddComponent<CameraCoverageMap>();

        Transform cameraTransform = yoloCamera != null ? yoloCamera.transform : null;
        coverageMap.Configure(cameraTransform, yoloCamera, transform);
    }

    private void EnsureArmContactTracker()
    {
        if (armRig == null)
            return;

        armContactTracker = armRig.GetComponent<ArmContactTracker>();
        if (armContactTracker == null)
            armContactTracker = armRig.gameObject.AddComponent<ArmContactTracker>();
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
        bool ballVisibleForPolicy = EvaluateBallVisibilityForPolicy();
        UpdateBallSightMemory(ballVisibleForPolicy);

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

        sensor.AddObservation(GetNoisyUltrasonicNormalized());
        sensor.AddObservation(virtualSensors != null ? virtualSensors.LeftIr : 0f);
        sensor.AddObservation(virtualSensors != null ? virtualSensors.RightIr : 0f);
        sensor.AddObservation(virtualSensors != null ? virtualSensors.GripperIr : 0f);
        sensor.AddObservation(GetReportedBallAngle(ballVisibleForPolicy));
        sensor.AddObservation(GetReportedBallDistance(ballVisibleForPolicy));
        sensor.AddObservation(lastKnownBallDirection * ballMemoryConfidence);
        sensor.AddObservation(ballVisibleForPolicy ? 1.0f : 0.0f);
        sensor.AddObservation(currentServoNormalize);
        sensor.AddObservation(isHoldingBall ? 1.0f : 0.0f);
        sensor.AddObservation(offsetFromStart.x);
        sensor.AddObservation(offsetFromStart.z);
        sensor.AddObservation(normalizedHeading);
        sensor.AddObservation(currentSpeed);
        sensor.AddObservation(Mathf.Clamp01(timeSinceLastDetection / ballMemoryDecaySeconds));
    }

    private float GetNoisyUltrasonicNormalized()
    {
        float normalized = virtualSensors != null ? virtualSensors.UltrasonicNormalized : 1f;
        if (!ShouldApplyDomainRandomization())
            return normalized;

        return Mathf.Clamp01(normalized + Random.Range(-0.05f, 0.05f));
    }

    private float GetReportedBallAngle(bool ballVisibleForPolicy)
    {
        if (ballVisibleForPolicy && yoloCamera != null)
            return yoloCamera.RelativeAngleX;

        return lastKnownBallAngle * ballMemoryConfidence;
    }

    private float GetReportedBallDistance(bool ballVisibleForPolicy)
    {
        if (ballVisibleForPolicy && yoloCamera != null)
            return yoloCamera.NormalizedDistance;

        return Mathf.Lerp(1f, lastKnownBallDistance, ballMemoryConfidence);
    }

    private void UpdateBallSightMemory(bool ballVisibleForPolicy)
    {
        if (yoloCamera == null)
            return;

        if (yoloCamera.IsBallVisible)
        {
            hasBallSightMemory = true;
            lastKnownBallAngle = yoloCamera.RelativeAngleX;
            lastKnownBallDistance = yoloCamera.NormalizedDistance;
            lastKnownBallDirection = Mathf.Abs(lastKnownBallAngle) < 0.05f
                ? 0f
                : Mathf.Sign(lastKnownBallAngle);
            ballMemoryConfidence = 1f;
            timeSinceLastDetection = 0f;
            return;
        }

        timeSinceLastDetection += Time.fixedDeltaTime;
        if (!hasBallSightMemory)
        {
            ballMemoryConfidence = 0f;
            return;
        }

        ballMemoryConfidence = Mathf.Clamp01(
            1f - (timeSinceLastDetection / ballMemoryDecaySeconds));

        if (ballMemoryConfidence <= 0.001f)
        {
            hasBallSightMemory = false;
            lastKnownBallDirection = 0f;
            lastKnownBallAngle = 0f;
            lastKnownBallDistance = 1f;
        }
    }

    private bool EvaluateBallVisibilityForPolicy()
    {
        if (yoloCamera == null)
            return false;

        yoloCamera.RefreshDetection();
        UpdateBurstDropout();
        return yoloCamera.IsBallVisible && burstDropoutRemaining <= 0;
    }

    private void UpdateBurstDropout()
    {
        if (burstDropoutRemaining > 0)
        {
            burstDropoutRemaining--;
            return;
        }

        if (!ShouldApplyDomainRandomization())
            return;

        float yawRateRadians = trackController != null
            ? Mathf.Abs(trackController.YawRateDegrees) * Mathf.Deg2Rad
            : (body != null ? body.angularVelocity.magnitude : 0f);

        if (yawRateRadians <= 0.5f)
            return;

        if (Random.value < 0.15f)
            burstDropoutRemaining = Random.Range(5, 16);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (gripperController != null && gripperController.IsHolding)
        {
            if (trackController != null)
                trackController.Stop(immediate: true);

            holdTicks++;
            AddReward(holdStepReward);
            if (holdTicks >= holdSuccessSteps)
            {
                AddReward(holdSuccessReward);
                episodeEndedInCatch = true;
                EndEpisode();
            }

            return;
        }

        if (maxEpisodeSteps > 0 && StepCount >= maxEpisodeSteps)
        {
            AddReward(episodeTimeoutPenalty);
            EndEpisode();
            return;
        }

        float moveCmd;
        float turnCmd;
        float servoPanCmd;
        float s1Cmd;
        float s2Cmd;

        if (ShouldApplyDomainRandomization() && currentActionLatency > 0)
        {
            float[] freshActions = BuildContinuousActionSnapshot(actions);
            actionBuffer.Enqueue(freshActions);
            float[] delayed = actionBuffer.Dequeue();
            moveCmd = delayed[0];
            turnCmd = delayed[1];
            servoPanCmd = delayed[2];
            s1Cmd = delayed.Length > 3 ? delayed[3] : 0f;
            s2Cmd = delayed.Length > 4 ? delayed[4] : 0f;
        }
        else
        {
            moveCmd = actions.ContinuousActions[0];
            turnCmd = actions.ContinuousActions[1];
            servoPanCmd = actions.ContinuousActions[2];
            s1Cmd = actions.ContinuousActions.Length > 3 ? actions.ContinuousActions[3] : 0f;
            s2Cmd = actions.ContinuousActions.Length > 4 ? actions.ContinuousActions[4] : 0f;
        }

        if (trackController != null)
            trackController.SetCommand(moveCmd, turnCmd);

        lastTurnCommand = turnCmd;
        lastServoPanCommand = servoPanCmd;

        ApplyCameraPan(servoPanCmd);
        ApplyArmCommands(s1Cmd, s2Cmd);
        ExecuteGripperAction(actions.DiscreteActions[0]);
        EvaluateRewards(moveCmd, turnCmd, servoPanCmd, s1Cmd, s2Cmd);
    }

    private float[] BuildContinuousActionSnapshot(ActionBuffers actions)
    {
        int width = ExpectedContinuousActions;
        float[] snapshot = new float[width];
        for (int index = 0; index < width; index++)
            snapshot[index] = actions.ContinuousActions[index];
        return snapshot;
    }

    private void ApplyCameraPan(float servoPanCmd)
    {
        if (sensorHeadRig == null)
            return;

        bool ballVisibleForPolicy = yoloCamera != null &&
                                    yoloCamera.IsBallVisible &&
                                    burstDropoutRemaining <= 0;
        float effectivePanCmd = servoPanCmd;
        float speedMultiplier = 1f;

        if (enableBallSearchAssist &&
            !ballVisibleForPolicy &&
            hasBallSightMemory &&
            ballMemoryConfidence > 0.05f &&
            Mathf.Abs(lastKnownBallDirection) > 0.05f)
        {
            speedMultiplier = searchCameraSpeedMultiplier;
            float searchNudge = lastKnownBallDirection *
                                searchAssistStrength *
                                ballMemoryConfidence;
            effectivePanCmd = Mathf.Clamp(effectivePanCmd + searchNudge, -1f, 1f);
        }

        const float baseDegreesPerSecond = 60f;
        float degreesPerSecond = baseDegreesPerSecond * speedMultiplier;
        float newPan = sensorHeadRig.S5PanAngle +
                       (effectivePanCmd * degreesPerSecond * Time.fixedDeltaTime);
        newPan = Mathf.Clamp(newPan, sensorHeadRig.S5MinimumAngle, sensorHeadRig.S5MaximumAngle);
        sensorHeadRig.SetPan(newPan);
    }

    private void ApplyArmCommands(float s1Cmd, float s2Cmd)
    {
        if (armRig == null || trainingMode != RobotTrainingMode.MobileArm)
            return;

        float engageGate = GetArmEngageGate();
        s1Cmd *= engageGate;
        s2Cmd *= engageGate;

        const float armDegreesPerSecond = 45f;
        float deltaTime = Time.fixedDeltaTime;
        float nextS1 = armRig.S1Angle + (s1Cmd * armDegreesPerSecond * deltaTime);
        float nextS2 = armRig.S2Angle + (s2Cmd * armDegreesPerSecond * deltaTime);

        // Hard ceiling so the policy cannot fold the claw into the sky.
        // Hard floor so the tip cannot dig into the ground and jack the chassis.
        float s1Ceiling = Mathf.Min(armRig.S1MaximumAngle, mobileTrainingS1Maximum);
        float s1Floor = Mathf.Max(armRig.S1MinimumAngle, mobileTrainingS1Minimum);
        float s2Floor = Mathf.Max(armRig.S2MinimumAngle, mobileTrainingS2Minimum);
        nextS1 = Mathf.Clamp(nextS1, s1Floor, s1Ceiling);
        nextS2 = Mathf.Clamp(nextS2, s2Floor, armRig.S2MaximumAngle);

        armRig.SetArmPoseLockedWrist(nextS1, nextS2, armRig.S4Closure);

        // Hard geometric guarantee: claw must never touch / penetrate the floor.
        if (!armRig.IsClawClearOfFloor)
        {
            armRig.EnforceFloorClearance();
            if (!armRig.IsClawClearOfFloor)
                AddReward(armFloorContactPenalty);
        }
    }

    private float GetDistanceToBallMetres()
    {
        if (yoloCamera == null || yoloCamera.targetBall == null)
            return float.PositiveInfinity;

        Vector3 robotPosition = body != null ? body.position : transform.position;
        return Vector3.Distance(robotPosition, yoloCamera.targetBall.position);
    }

    /// <summary>
    /// 0 when far from the ball, 1 when close enough to begin arm engagement.
    /// </summary>
    private float GetArmEngageGate()
    {
        float distance = GetDistanceToBallMetres();
        if (float.IsPositiveInfinity(distance))
            return 0f;

        float span = Mathf.Max(0.05f, armEngageFarDistance - armEngageNearDistance);
        return 1f - Mathf.Clamp01((distance - armEngageNearDistance) / span);
    }

    private void EvaluateRewards(
        float moveCommand,
        float turnCommand,
        float servoPanCommand,
        float s1Command,
        float s2Command)
    {
        AddReward(timePenalty);

        float turnMagnitude = Mathf.Abs(turnCommand);
        float moveMagnitude = Mathf.Abs(moveCommand);
        float robotSpeed = trackController != null ? Mathf.Abs(trackController.LinearSpeed) : 0f;
        float yawRate = trackController != null ? Mathf.Abs(trackController.YawRateDegrees) : 0f;

        AddReward(bodyTurnPenalty * turnMagnitude);

        if (turnMagnitude > 0.28f && moveMagnitude < 0.14f)
            AddReward(spinInPlacePenalty * turnMagnitude);

        if (yawRate > spinInPlaceYawThreshold && robotSpeed < spinInPlaceSpeedThreshold)
        {
            float spinIntensity = Mathf.Clamp01(
                (yawRate - spinInPlaceYawThreshold) / 90f);
            AddReward(spinInPlacePenalty * (0.6f + spinIntensity));
        }

        if (moveCommand < -0.04f)
        {
            float reverseMagnitude = -moveCommand;
            AddReward(reverseDrivePenalty * reverseMagnitude);
        }

        bool isHoldingBall = gripperController != null && gripperController.IsHolding;
        bool ballVisible = yoloCamera != null && yoloCamera.IsBallVisible && burstDropoutRemaining <= 0;

        if (moveCommand < -0.04f && ballVisible)
            AddReward(reverseWhileBallVisiblePenalty * -moveCommand);

        if (ballVisible)
        {
            float ballOffCenter = Mathf.Abs(yoloCamera.RelativeAngleX);
            float sightDampen = 1f - Mathf.Clamp01(yawRate / 70f) * 0.9f;
            AddReward(ballInSightReward * sightDampen);
            float centerBonus = 0.002f * (1.0f - ballOffCenter) * sightDampen;
            AddReward(centerBonus);

            float panMagnitude = Mathf.Abs(servoPanCommand);
            if (ballOffCenter > 0.12f && panMagnitude > 0.04f)
                AddReward(cameraPanReward * panMagnitude * Mathf.Clamp01(ballOffCenter / 0.5f));

            if (ballOffCenter > 0.15f && turnMagnitude > 0.1f)
            {
                float bodyTurnScale = CanCameraStillReachBall(yoloCamera.RelativeAngleX) ? 2.4f : 0.35f;
                AddReward(bodyTurnWhileBallOffCenterPenalty * turnMagnitude * bodyTurnScale);
            }

            if (ballOffCenter < 0.1f && turnMagnitude > 0.12f)
            {
                AddReward(
                    bodyTurnWhileBallCenteredPenalty *
                    turnMagnitude *
                    (1f + Mathf.Clamp01(yawRate / 60f)));
            }
        }
        else if (hasBallSightMemory && ballMemoryConfidence > 0.1f)
        {
            float panMagnitude = Mathf.Abs(servoPanCommand);
            if (panMagnitude > 0.05f && Mathf.Abs(lastKnownBallDirection) > 0.05f)
            {
                float panTowardMemory = servoPanCommand * lastKnownBallDirection;
                if (panTowardMemory > 0f)
                    AddReward(searchPanReward * panTowardMemory * ballMemoryConfidence);
            }
        }

        EvaluateCameraAntiLoopPenalties(servoPanCommand);

        float armEngageGate = GetArmEngageGate();

        if (yoloCamera != null && yoloCamera.targetBall != null)
        {
            Vector3 robotPosition = body != null ? body.position : transform.position;
            float currentDistance = Vector3.Distance(robotPosition, yoloCamera.targetBall.position);
            Rigidbody ballBody = yoloCamera.targetBall.GetComponent<Rigidbody>();
            float ballSpeed = ballBody != null ? ballBody.linearVelocity.magnitude : 0f;

            if (previousDistanceToBall < float.MaxValue)
            {
                float distanceDelta = previousDistanceToBall - currentDistance;
                float approachScale = distanceShapingScale;
                if (currentDistance < closeApproachDistance)
                {
                    approachScale *= Mathf.Lerp(0.55f, 1f, currentDistance / closeApproachDistance);
                    if (robotSpeed > 0.12f)
                        approachScale *= 0.45f;
                }

                AddReward(distanceDelta * approachScale);
            }

            previousDistanceToBall = currentDistance;

            if (!isHoldingBall && currentDistance < closeApproachDistance)
            {
                if (robotSpeed > 0.11f && moveMagnitude > 0.55f)
                    AddReward(rushNearBallPenalty * Mathf.Max(robotSpeed / 0.18f, moveMagnitude));

                if (robotSpeed > 0.02f &&
                    robotSpeed < 0.11f &&
                    currentDistance < closeApproachDistance * 0.95f)
                {
                    AddReward(slowApproachReward);
                }

                if (previousBallSpeed < float.MaxValue &&
                    ballSpeed > previousBallSpeed + 0.04f &&
                    robotSpeed > 0.04f)
                {
                    AddReward(ballKickPenalty * (ballSpeed - previousBallSpeed));
                }
            }

            previousBallSpeed = ballSpeed;

            if (trainingMode == RobotTrainingMode.MobileArm &&
                ballVisible &&
                armRig != null &&
                armRig.HoldPoint != null &&
                armEngageGate > 0.05f)
            {
                float holdDistance = Vector3.Distance(
                    armRig.HoldPoint.position,
                    yoloCamera.targetBall.position);
                if (previousHoldPointDistance < float.MaxValue)
                {
                    AddReward(
                        (previousHoldPointDistance - holdDistance) *
                        0.6f *
                        armEngageGate);
                }

                previousHoldPointDistance = holdDistance;
            }
            else if (armEngageGate <= 0.05f)
            {
                previousHoldPointDistance = float.MaxValue;
            }
        }

        if (trainingMode == RobotTrainingMode.MobileArm && armRig != null)
        {
            EvaluateMobileArmPoseRewards(armEngageGate, s1Command, s2Command);
        }

        if (virtualSensors != null)
        {
            bool obstacleNearby = virtualSensors.LeftIrDetected ||
                                  virtualSensors.RightIrDetected ||
                                  virtualSensors.UltrasonicNormalized < 0.12f;
            if (obstacleNearby)
            {
                float penalty = obstacleProximityPenalty;
                if (moveMagnitude > 0.2f)
                    penalty *= 1f + moveMagnitude;
                AddReward(penalty);
            }
        }

        if (armContactTracker != null && armContactTracker.HasRecentContact)
            AddReward(armWallContactPenalty);

        if (armContactTracker != null && armContactTracker.HasRecentFloorContact)
            AddReward(armFloorContactPenalty);

        EvaluateCoverageRewards(servoPanCommand);

        if (virtualSensors != null &&
            virtualSensors.GripperIrDetected &&
            gripperController != null &&
            !gripperController.IsHolding)
        {
            AddReward(gripperIrReward);
        }
    }

    private void EvaluateCameraAntiLoopPenalties(float servoPanCommand)
    {
        if (sensorHeadRig == null)
            return;

        float panMagnitude = Mathf.Abs(servoPanCommand);
        if (panMagnitude > 0.18f && IsCameraPanAtLimit(servoPanCommand))
            AddReward(cameraPanAtLimitPenalty * panMagnitude);

        if (panMagnitude > 0.22f && Mathf.Abs(previousServoPanCommand) > 0.22f)
        {
            float oscillation = Mathf.Min(panMagnitude, Mathf.Abs(previousServoPanCommand));
            if (Mathf.Sign(servoPanCommand) != Mathf.Sign(previousServoPanCommand))
                AddReward(cameraPanOscillationPenalty * oscillation);
        }

        previousServoPanCommand = servoPanCommand;
    }

    private bool IsCameraPanAtLimit(float panCommand)
    {
        if (sensorHeadRig == null || Mathf.Abs(panCommand) < 0.08f)
            return false;

        const float limitSlackDegrees = 2f;
        if (panCommand > 0f)
        {
            return sensorHeadRig.S5PanAngle >=
                   sensorHeadRig.S5MaximumAngle - limitSlackDegrees;
        }

        return sensorHeadRig.S5PanAngle <=
               sensorHeadRig.S5MinimumAngle + limitSlackDegrees;
    }

    private void EvaluateCoverageRewards(float servoPanCommand)
    {
        if (!enableCoverageRewards || coverageMap == null)
            return;

        float panMagnitude = Mathf.Abs(servoPanCommand);
        float turnMagnitude = Mathf.Abs(lastTurnCommand);
        float yawRate = trackController != null ? Mathf.Abs(trackController.YawRateDegrees) : 0f;
        if (panMagnitude > 0.04f &&
            turnMagnitude < 0.22f &&
            yawRate < spinInPlaceYawThreshold &&
            coverageMap.LastRevealColdness > 0.05f)
        {
            AddReward(
                coverageRevealReward *
                coverageMap.LastRevealColdness *
                panMagnitude);
        }

        if (Academy.IsInitialized)
        {
            Academy.Instance.StatsRecorder.Add(
                "GFSX/Coverage/Mean",
                coverageMap.MeanWarmth,
                StatAggregationMethod.Average);
        }
    }

    private bool CanCameraStillReachBall(float ballAngleX)
    {
        if (sensorHeadRig == null)
            return false;

        const float limitSlackDegrees = 2f;
        if (ballAngleX > 0.08f)
            return sensorHeadRig.S5PanAngle < sensorHeadRig.S5MaximumAngle - limitSlackDegrees;

        if (ballAngleX < -0.08f)
            return sensorHeadRig.S5PanAngle > sensorHeadRig.S5MinimumAngle + limitSlackDegrees;

        return false;
    }

    /// <summary>
    /// Pose shaping / penalties for the controllable arm. FixedArm skips this entirely.
    /// </summary>
    private void EvaluateMobileArmPoseRewards(
        float armEngageGate,
        float s1Command,
        float s2Command)
    {
        float armCommandMagnitude = Mathf.Abs(s1Command) + Mathf.Abs(s2Command);
        float armJointDelta =
            Mathf.Abs(armRig.S1Angle - previousS1Angle) +
            Mathf.Abs(armRig.S2Angle - previousS2Angle);

        if (armEngageGate < 0.35f && armJointDelta > 0.01f)
            AddReward(farArmMotionPenalty * armJointDelta);

        if (armEngageGate > 0.55f && armCommandMagnitude > 0.05f)
            AddReward(closeArmMotionReward * armEngageGate * armCommandMagnitude);

        // Raised shoulder (claw toward sky / blocks sensors).
        float s1AbovePickup = armRig.S1Angle - armRig.FloorPickupS1;
        if (s1AbovePickup > highArmS1SlackDegrees)
        {
            float highFactor = Mathf.Clamp01(
                (s1AbovePickup - highArmS1SlackDegrees) / 45f);
            // Stronger while navigating / far from the ball.
            float farBias = 1f - 0.55f * armEngageGate;
            AddReward(highArmPenalty * highFactor * farBias);
        }

        // Diving below floor-pickup scrapes the ground / tips the chassis.
        float s1BelowPickup = armRig.FloorPickupS1 - armRig.S1Angle;
        if (s1BelowPickup > 1.5f)
        {
            float diveFactor = Mathf.Clamp01(s1BelowPickup / 12f);
            AddReward(lowArmDivePenalty * diveFactor);
        }

        // Soft preference for floor-ready S1/S2 while far from the ball.
        if (armEngageGate < 0.4f)
        {
            float s1Error = Mathf.Abs(armRig.S1Angle - armRig.FloorPickupS1) / 55f;
            float s2Error = Mathf.Abs(armRig.S2Angle - armRig.FloorPickupS2) / 75f;
            float poseError = Mathf.Clamp01(0.65f * s1Error + 0.35f * s2Error);
            AddReward(floorPoseBiasReward * (1f - poseError) * (1f - armEngageGate));
        }

        Transform hold = armRig.HoldPoint;
        if (hold != null)
        {
            float pitchUp = hold.forward.y;
            if (pitchUp > skyClawPitchThreshold)
            {
                float skyFactor = Mathf.Clamp01(
                    (pitchUp - skyClawPitchThreshold) / (1f - skyClawPitchThreshold));
                AddReward(skyClawPitchPenalty * skyFactor);
            }

            float holdHeight = hold.position.y;
            if (holdHeight > highHoldPointHeightMetres)
            {
                float heightFactor = Mathf.Clamp01(
                    (holdHeight - highHoldPointHeightMetres) / 0.25f);
                AddReward(highHoldPointPenalty * heightFactor);
            }
        }

        if (virtualSensors != null &&
            (virtualSensors.LeftIrOccludedByArm || virtualSensors.RightIrOccludedByArm))
        {
            // Occlusion is worse when the arm is high / far from grasp.
            float occlusionScale = 1f + Mathf.Clamp01(
                (armRig.S1Angle - armRig.FloorPickupS1) / 40f);
            AddReward(armOcclusionPenalty * occlusionScale);
        }

        previousS1Angle = armRig.S1Angle;
        previousS2Angle = armRig.S2Angle;
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
                if (armRig.IsJawClosed)
                    break;

                // Claw may only click when the ball is inside and IR sees it.
                bool irSeesBall = virtualSensors != null && virtualSensors.GripperIrDetected;
                if (!irSeesBall)
                {
                    AddReward(emptyGripperClosePenalty);
                    break;
                }

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

        // Only auto-close from a truly open claw (open→close edge).
        if (!gripperController.AreJawsOpen && !gripperController.IsGrabArmed)
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
