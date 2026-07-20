using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class RobotBrain : Agent
{
    private const int DriveContinuousActions = 3;
    private const int MobileArmContinuousActions = 6;

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
    [SerializeField] private float timePenalty = -0.001f;
    [SerializeField] private float ballInSightReward = 0.004f;
    [SerializeField] private float gripperIrReward = 0.01f;
    [SerializeField] private float emptyGripperClosePenalty = -0.06f;
    [SerializeField] private float armOcclusionPenalty = -0.008f;
    [SerializeField] private float holdStepReward = 0.02f;
    [SerializeField] private float holdSuccessReward = 3.0f;
    [SerializeField] private int holdSuccessSteps = 50;
    [SerializeField] private float bodyTurnPenalty = -0.002f;
    [SerializeField] private float cameraPanReward = 0.003f;
    [SerializeField] private float bodyTurnWhileBallOffCenterPenalty = -0.005f;
    [SerializeField] private float farArmMotionPenalty = -0.003f;
    [SerializeField] private float closeArmMotionReward = 0.004f;
    [SerializeField] private float rushNearBallPenalty = -0.018f;
    [SerializeField] private float ballKickPenalty = -0.12f;
    [SerializeField] private float slowApproachReward = 0.005f;
    [SerializeField] private float closeApproachDistance = 0.55f;
    [SerializeField] private float distanceShapingScale = 0.55f;

    [Header("Arm engagement (MobileArm)")]
    [Tooltip("Arm commands are scaled down to zero beyond this distance (metres).")]
    [SerializeField] private float armEngageFarDistance = 1.8f;
    [Tooltip("Arm commands reach full strength inside this distance (metres).")]
    [SerializeField] private float armEngageNearDistance = 0.85f;

    [Header("Gripper Assistance")]
    [SerializeField] private bool autoCloseGripperOnBallDetect = true;

    [Header("Ball sight memory & camera search")]
    [SerializeField] private bool enableBallSearchAssist = true;
    [Tooltip("Extra camera pan speed while the ball is off-screen but still remembered.")]
    [SerializeField, Range(1f, 2f)] private float searchCameraSpeedMultiplier = 1.35f;
    [Tooltip("Small assist added to pan command toward the last seen ball direction.")]
    [SerializeField, Range(0f, 0.5f)] private float searchAssistStrength = 0.22f;
    [Tooltip("How long the last sighting stays useful before fading out (seconds).")]
    [SerializeField, Min(0.5f)] private float ballMemoryDecaySeconds = 4f;
    [SerializeField] private float searchPanReward = 0.003f;

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
    private float previousS1Angle;
    private float previousS2Angle;
    private float previousS3Angle;

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
        if (trackController != null) trackController.Stop(immediate: true);

        if (gripperController != null && gripperController.IsHolding)
            gripperController.Release();

        if (sensorHeadRig != null) sensorHeadRig.ResetToNeutral();
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
        CacheArmAngleBaselines();
    }

    private void CacheArmAngleBaselines()
    {
        if (armRig == null)
            return;

        previousS1Angle = armRig.S1Angle;
        previousS2Angle = armRig.S2Angle;
        previousS3Angle = armRig.S3Angle;
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
                EndEpisode();
            }

            return;
        }

        float moveCmd;
        float turnCmd;
        float servoPanCmd;
        float s1Cmd;
        float s2Cmd;
        float s3Cmd;

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
            s3Cmd = delayed.Length > 5 ? delayed[5] : 0f;
        }
        else
        {
            moveCmd = actions.ContinuousActions[0];
            turnCmd = actions.ContinuousActions[1];
            servoPanCmd = actions.ContinuousActions[2];
            s1Cmd = actions.ContinuousActions.Length > 3 ? actions.ContinuousActions[3] : 0f;
            s2Cmd = actions.ContinuousActions.Length > 4 ? actions.ContinuousActions[4] : 0f;
            s3Cmd = actions.ContinuousActions.Length > 5 ? actions.ContinuousActions[5] : 0f;
        }

        if (trackController != null)
            trackController.SetCommand(moveCmd, turnCmd);

        lastTurnCommand = turnCmd;
        lastServoPanCommand = servoPanCmd;

        ApplyCameraPan(servoPanCmd);
        ApplyArmCommands(s1Cmd, s2Cmd, s3Cmd);
        ExecuteGripperAction(actions.DiscreteActions[0]);
        EvaluateRewards(moveCmd, turnCmd, servoPanCmd, s1Cmd, s2Cmd, s3Cmd);
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

    private void ApplyArmCommands(float s1Cmd, float s2Cmd, float s3Cmd)
    {
        if (armRig == null || trainingMode != RobotTrainingMode.MobileArm)
            return;

        float engageGate = GetArmEngageGate();
        s1Cmd *= engageGate;
        s2Cmd *= engageGate;
        s3Cmd *= engageGate;

        const float armDegreesPerSecond = 45f;
        float deltaTime = Time.fixedDeltaTime;
        float nextS1 = armRig.S1Angle + (s1Cmd * armDegreesPerSecond * deltaTime);
        float nextS2 = armRig.S2Angle + (s2Cmd * armDegreesPerSecond * deltaTime);
        float nextS3 = armRig.S3Angle + (s3Cmd * armDegreesPerSecond * deltaTime);
        armRig.SetServoCommands(nextS1, nextS2, nextS3, armRig.S4Closure);
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
        float s2Command,
        float s3Command)
    {
        AddReward(timePenalty);

        float turnMagnitude = Mathf.Abs(turnCommand);
        float moveMagnitude = Mathf.Abs(moveCommand);
        AddReward(bodyTurnPenalty * turnMagnitude);

        bool isHoldingBall = gripperController != null && gripperController.IsHolding;
        bool ballVisible = yoloCamera != null && yoloCamera.IsBallVisible && burstDropoutRemaining <= 0;
        if (ballVisible)
        {
            AddReward(ballInSightReward);
            float centerBonus = 0.005f * (1.0f - Mathf.Abs(yoloCamera.RelativeAngleX));
            AddReward(centerBonus);

            float panMagnitude = Mathf.Abs(servoPanCommand);
            if (Mathf.Abs(yoloCamera.RelativeAngleX) > 0.15f && panMagnitude > 0.05f)
                AddReward(cameraPanReward * panMagnitude);

            if (Mathf.Abs(yoloCamera.RelativeAngleX) > 0.2f && turnMagnitude > 0.15f)
                AddReward(bodyTurnWhileBallOffCenterPenalty * turnMagnitude);
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

        float armEngageGate = GetArmEngageGate();

        if (yoloCamera != null && yoloCamera.targetBall != null)
        {
            Vector3 robotPosition = body != null ? body.position : transform.position;
            float currentDistance = Vector3.Distance(robotPosition, yoloCamera.targetBall.position);
            float robotSpeed = trackController != null ? Mathf.Abs(trackController.LinearSpeed) : 0f;
            Rigidbody ballBody = yoloCamera.targetBall.GetComponent<Rigidbody>();
            float ballSpeed = ballBody != null ? ballBody.linearVelocity.magnitude : 0f;

            if (previousDistanceToBall < float.MaxValue)
            {
                float distanceDelta = previousDistanceToBall - currentDistance;
                float approachScale = distanceShapingScale;
                if (currentDistance < closeApproachDistance)
                {
                    approachScale *= Mathf.Clamp01(currentDistance / closeApproachDistance);
                    if (robotSpeed > 0.08f)
                        approachScale *= 0.25f;
                }

                AddReward(distanceDelta * approachScale);
            }

            previousDistanceToBall = currentDistance;

            if (!isHoldingBall && currentDistance < closeApproachDistance)
            {
                if (robotSpeed > 0.07f || moveMagnitude > 0.35f)
                    AddReward(rushNearBallPenalty * Mathf.Max(robotSpeed / 0.15f, moveMagnitude));

                if (robotSpeed < 0.05f && currentDistance < closeApproachDistance * 0.85f)
                    AddReward(slowApproachReward);

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
                        1.5f *
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
            float armCommandMagnitude =
                Mathf.Abs(s1Command) + Mathf.Abs(s2Command) + Mathf.Abs(s3Command);
            float armJointDelta =
                Mathf.Abs(armRig.S1Angle - previousS1Angle) +
                Mathf.Abs(armRig.S2Angle - previousS2Angle) +
                Mathf.Abs(armRig.S3Angle - previousS3Angle);

            if (armEngageGate < 0.35f && armJointDelta > 0.01f)
                AddReward(farArmMotionPenalty * armJointDelta);

            if (armEngageGate > 0.55f && armCommandMagnitude > 0.05f)
                AddReward(closeArmMotionReward * armEngageGate * armCommandMagnitude);

            previousS1Angle = armRig.S1Angle;
            previousS2Angle = armRig.S2Angle;
            previousS3Angle = armRig.S3Angle;
        }

        if (virtualSensors != null)
        {
            if (virtualSensors.LeftIrDetected ||
                virtualSensors.RightIrDetected ||
                virtualSensors.UltrasonicNormalized < 0.1f)
            {
                AddReward(-0.008f);
            }

            if (virtualSensors.LeftIrOccludedByArm || virtualSensors.RightIrOccludedByArm)
                AddReward(armOcclusionPenalty);
        }

        if (virtualSensors != null &&
            virtualSensors.GripperIrDetected &&
            gripperController != null &&
            !gripperController.IsHolding)
        {
            AddReward(gripperIrReward);
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
                if (!armRig.IsJawClosed)
                {
                    if (virtualSensors != null && !virtualSensors.GripperIrDetected)
                        AddReward(emptyGripperClosePenalty);
                    armRig.SetJawClosed();
                }
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
