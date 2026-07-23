using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Fail-closed action sink for the physical robot. Policy outputs are always
/// visible in dry-run mode, but real hardware starts disarmed every Play.
/// </summary>
[DefaultExecutionOrder(300)]
[DisallowMultipleComponent]
public sealed class GfsxPhysicalActionRouter : MonoBehaviour
{
    private const string CmdVelTopic = "/gfsx/cmd_vel";
    private const string DriveEnableTopic = "/gfsx/drive_enable";
    private const string ServoTopic = "/gfsx/servo_targets_degrees";
    private const string ServoEnableTopic = "/gfsx/servo_enable";
    private const int ServoCount = 6;
    private const int ShoulderServoIndex = 0;
    private const int ElbowServoIndex = 1;
    private const int WristServoIndex = 2;
    private const int ClawServoIndex = 3;
    private const int CameraPanServoIndex = 4;
    private const int CameraTiltServoIndex = 5;
    // The Pi bridge accepts the full calibrated S1-S3 servo range. S5 keeps
    // its separate 15-160 degree limit below.
    private const float FixedArmMinimumDegrees = 0f;
    private const float FixedArmMaximumDegrees = 180f;
    // Must match TURN_K in the Raspberry Pi bridge. This converts the
    // already-limited wheel pair back into the Twist contract used on wire.
    private const float PhysicalBridgeTurnGain = 0.25f;
    private const float ConfirmationWindowSeconds = 8f;
    private const float MinimumServoDisarmDwellSeconds = 1.1f;

    [Header("Physical inputs")]
    [SerializeField] private GfsxPhysicalTelemetry telemetry;
    [SerializeField] private GfsxRealVisionReceiver vision;

    [Header("Output permissions")]
    [Tooltip("Still requires a two-step runtime arm. It never persists between Play sessions.")]
    [SerializeField] private bool allowDriveArming = false;
    [Tooltip("Editor smoke-test escape hatch. When true, no physical ROS publisher is registered or used.")]
    [SerializeField] private bool suppressAllPhysicalPublishing = false;
    [Tooltip("S5 pan is changed only after its own two-step runtime arm.")]
    [SerializeField] private bool allowCameraPanServo = true;
    [Tooltip("Makes the separate S4/F7 arm available. Runtime still rejects an uncalibrated S4 state.")]
    [SerializeField] private bool allowClawServoActions = false;
    [Tooltip("Makes the separate fixed S1-S3/F6 authorization available. This does not add policy actions.")]
    [SerializeField] private bool allowFixedArmPose = false;
    [Tooltip("Blocks track arming until the fixed S1-S3 command estimate has reached its latched targets.")]
    [SerializeField] private bool requireFixedArmPoseBeforeDrive = true;

    [Header("Heartbeat and stale-data gates")]
    [SerializeField, Range(5f, 12f)] private float publishRateHz = 8f;
    [SerializeField, Range(0.15f, 1f)] private float actionTimeoutSeconds = 0.35f;

    [Header("Training-equivalent drive mixer")]
    [SerializeField, Min(0.01f)] private float moveSpeed = 0.57f;
    [SerializeField, Min(1f)] private float turnSpeedDegrees = 120f;
    [SerializeField, Range(0f, 1f)] private float turnContribution = 0.30f;
    [SerializeField, Min(0.01f)] private float maximumLinearCommand = 0.25f;
    [SerializeField, Min(1f)] private float pwmPerMetrePerSecond = 200f;
    [SerializeField, Range(0f, 99f)] private float motorDeadZone = 10f;
    [SerializeField, Range(0f, 100f)] private float minimumMovingPwm = 35f;
    [SerializeField, Range(0.1f, 100f)] private float maximumPwmStep = 15f;
    [SerializeField, Range(35f, 100f)] private float maximumTrackPwm = 35f;

    [Header("Independent physical safety overrides")]
    [SerializeField, Range(0.05f, 0.5f)] private float forwardStopDistanceMetres = 0.12f;
    [SerializeField] private bool stopForwardOnFrontIr = true;
    [Tooltip("Reverse requires the installed IO4 rear IR to be fresh and clear.")]
    [SerializeField] private bool requireFreshRearIrForReverse = true;

    [Header("S5 / optional repaired S4")]
    [SerializeField] private float sensorPanMinimumDegrees = 15f;
    [SerializeField] private float sensorPanMaximumDegrees = 160f;
    [SerializeField, Min(1f)] private float sensorPanDegreesPerSecond = 45f;
    [Tooltip("Maps positive policy pan to physical degrees. The Fixed model uses -1.")]
    [SerializeField] private float sensorPanActionDirection = 1f;
    [SerializeField] private float clawOpenDegrees = 0f;
    [SerializeField] private float clawClosedDegrees = 50f;
    [SerializeField, Range(0.1f, 3f)] private float clawCommandCooldownSeconds = 0.5f;

    [Header("Fixed S1-S3 plus S6 pose (absolute physical degrees)")]
    [Tooltip("Training-equivalent starting value. Confirm the real shoulder pose mechanically before use.")]
    [SerializeField, Range(FixedArmMinimumDegrees, FixedArmMaximumDegrees)]
    private float fixedArmS1Degrees = 70f;
    [Tooltip("Training-equivalent starting value. Confirm the real elbow pose mechanically before use.")]
    [SerializeField, Range(FixedArmMinimumDegrees, FixedArmMaximumDegrees)]
    private float fixedArmS2Degrees = 180f;
    [Tooltip("Training-equivalent starting value. Confirm the real wrist pose mechanically before use.")]
    [SerializeField, Range(FixedArmMinimumDegrees, FixedArmMaximumDegrees)]
    private float fixedArmS3Degrees = 90f;
    [Tooltip("Fixed camera pitch used by the Fixed-policy training scene.")]
    [SerializeField, Range(FixedArmMinimumDegrees, FixedArmMaximumDegrees)]
    private float fixedCameraTiltDegrees = 75f;
    [Tooltip("Tolerance applies to the Pi command estimate, not encoder-measured shaft position.")]
    [SerializeField, Range(0.5f, 5f)] private float fixedArmToleranceDegrees = 1f;

    [Header("Runtime state (read-only while playing)")]
    [SerializeField] private bool driveArmed;
    [SerializeField] private bool cameraServosRequested;
    [SerializeField] private bool cameraServosArmed;
    [SerializeField] private bool clawServoRequested;
    [SerializeField] private bool clawServoArmed;
    [SerializeField] private bool fixedArmPoseRequested;
    [SerializeField] private bool fixedArmPoseArmed;
    [SerializeField] private bool fixedArmPoseAtTarget;
    [SerializeField] private string safetyStatus = "DRY RUN";

    private readonly float[] servoTargets = new float[ServoCount];
    private readonly float[] observedServoState = new float[ServoCount];
    private ROSConnection ros;
    private bool publishersRegistered;
    private float nextPublishTime;
    private float latestGas;
    private float latestSteering;
    private float latestCameraPan;
    private float latestCameraPanSpeedMultiplier = 1f;
    private int latestClawCommand;
    private float lastActionTime = float.NegativeInfinity;
    private int receivedActionCount;
    private float leftPwm;
    private float rightPwm;
    private float mixedLinearSpeed;
    private float mixedYawRateDegrees;
    private bool panTargetInitialized;
    private float panTargetDegrees;
    private bool clawTargetInitialized;
    private float clawTargetDegrees;
    private bool clawClosed;
    private float driveConfirmationDeadline;
    private float servoConfirmationDeadline;
    private float servoConfirmationReadyAt;
    private float clawConfirmationDeadline;
    private float clawConfirmationReadyAt;
    private float fixedArmConfirmationDeadline;
    private float fixedArmConfirmationReadyAt;
    private float fixedArmCandidateS1Degrees;
    private float fixedArmCandidateS2Degrees;
    private float fixedArmCandidateS3Degrees;
    private float fixedCameraTiltCandidateDegrees;
    private float fixedArmLatchedS1Degrees;
    private float fixedArmLatchedS2Degrees;
    private float fixedArmLatchedS3Degrees;
    private float fixedCameraTiltLatchedDegrees;
    private bool fixedArmTargetsLatched;
    private int clawArmActionCount;
    private int lastProcessedClawActionCount;
    private float lastClawTargetChangeTime = float.NegativeInfinity;
    private bool servoGateWasArmed;
    private bool forwardOverrideActive;
    private bool reverseOverrideActive;
    private Vector2 hudScroll;

    public bool DriveArmed => driveArmed;
    public bool CameraServosArmed => cameraServosArmed;
    public bool ClawServoArmed => clawServoArmed;
    public bool FixedArmPoseArmed => fixedArmPoseArmed;
    public bool FixedArmPoseAtTarget => fixedArmPoseAtTarget;
    public bool AllowClawServoActions => allowClawServoActions;
    public bool AllowFixedArmPose => allowFixedArmPose;
    public bool RequireFixedArmPoseBeforeDrive =>
        requireFixedArmPoseBeforeDrive;
    public bool FixedArmTargetsWithinLimits =>
        ValidFixedArmTarget(fixedArmS1Degrees) &&
        ValidFixedArmTarget(fixedArmS2Degrees) &&
        ValidFixedArmTarget(fixedArmS3Degrees) &&
        ValidFixedArmTarget(fixedCameraTiltDegrees);
    public float FixedArmS1Degrees => fixedArmS1Degrees;
    public float FixedArmS2Degrees => fixedArmS2Degrees;
    public float FixedArmS3Degrees => fixedArmS3Degrees;
    public float FixedCameraTiltDegrees => fixedCameraTiltDegrees;
    public float SensorPanDegreesPerSecond => sensorPanDegreesPerSecond;
    public float SensorPanActionDirection => sensorPanActionDirection;
    public bool AllowDriveArming => allowDriveArming;
    public bool PhysicalPublishingSuppressed =>
        suppressAllPhysicalPublishing;
    public string SafetyStatus => safetyStatus;
    public int ReceivedActionCount => receivedActionCount;
    public float LatestGas => latestGas;
    public float LatestSteering => latestSteering;
    public float LatestCameraPan => latestCameraPan;
    public int LatestClawCommand => latestClawCommand;
    public bool ForwardOverrideActive => forwardOverrideActive;
    public bool ReverseOverrideActive => reverseOverrideActive;
    public bool HasBall => clawServoArmed && clawClosed &&
        telemetry != null && telemetry.GripperIr > 0.5f;
    private bool ServoGateRequested =>
        cameraServosRequested || clawServoRequested || fixedArmPoseRequested;
    public bool HasFreshPolicyAction =>
        receivedActionCount > 0 &&
        Time.realtimeSinceStartup - lastActionTime <= actionTimeoutSeconds;
    public bool ReadyForPhysicalDrive =>
        allowDriveArming && telemetry != null && vision != null &&
        telemetry.RosConnected && telemetry.SensorFresh &&
        (!requireFreshRearIrForReverse || telemetry.RearIrFresh) &&
        vision.HasFreshPacket &&
        (!requireFixedArmPoseBeforeDrive ||
         (fixedArmPoseArmed && fixedArmPoseAtTarget)) &&
        HasFreshPolicyAction;

    private void Reset()
    {
        ResolveReferences();
        ResetRuntimeState();
    }

    private void Awake()
    {
        ResolveReferences();
        ResetRuntimeState();
        ros = ROSConnection.GetOrCreateInstance();
        if (!suppressAllPhysicalPublishing)
            EnsurePublishers();
    }

    private void OnEnable()
    {
        if (Application.isPlaying)
            ResetRuntimeState();
    }

    private void Update()
    {
        ReadEmergencyAndArmKeys();
        RefreshSafetyState();
        if (Time.unscaledTime < nextPublishTime)
            return;
        nextPublishTime = Time.unscaledTime + 1f / publishRateHz;
        PublishHeartbeatAndCommands();
    }

    private void FixedUpdate()
    {
        UpdateDriveMixer();
        UpdateServoTargets();
    }

    private void OnDisable()
    {
        EmergencyStop("Component disabled: physical output stopped.");
    }

    private void OnApplicationQuit()
    {
        EmergencyStop("Unity exiting: physical output stopped.");
    }

    private void OnValidate()
    {
        ResolveReferences();
        publishRateHz = Mathf.Clamp(publishRateHz, 5f, 12f);
        actionTimeoutSeconds = Mathf.Clamp(actionTimeoutSeconds, 0.15f, 1f);
        moveSpeed = Mathf.Max(0.01f, moveSpeed);
        turnSpeedDegrees = Mathf.Max(1f, turnSpeedDegrees);
        turnContribution = Mathf.Clamp01(turnContribution);
        maximumLinearCommand = Mathf.Max(0.01f, maximumLinearCommand);
        pwmPerMetrePerSecond = Mathf.Max(1f, pwmPerMetrePerSecond);
        motorDeadZone = Mathf.Clamp(motorDeadZone, 0f, 99f);
        minimumMovingPwm = Mathf.Clamp(
            Mathf.Max(minimumMovingPwm, motorDeadZone), 0f, maximumTrackPwm);
        maximumPwmStep = Mathf.Clamp(maximumPwmStep, 0.1f, 100f);
        maximumTrackPwm = Mathf.Clamp(maximumTrackPwm, 35f, 100f);
        forwardStopDistanceMetres = Mathf.Clamp(
            forwardStopDistanceMetres, 0.05f, 0.5f);
        if (sensorPanMaximumDegrees < sensorPanMinimumDegrees)
        {
            float swap = sensorPanMinimumDegrees;
            sensorPanMinimumDegrees = sensorPanMaximumDegrees;
            sensorPanMaximumDegrees = swap;
        }
        clawOpenDegrees = Mathf.Clamp(clawOpenDegrees, 0f, 180f);
        clawClosedDegrees = Mathf.Clamp(clawClosedDegrees, 0f, 180f);
        clawCommandCooldownSeconds = Mathf.Clamp(
            clawCommandCooldownSeconds, 0.1f, 3f);
        fixedArmS1Degrees = ClampFixedArmTarget(fixedArmS1Degrees);
        fixedArmS2Degrees = ClampFixedArmTarget(fixedArmS2Degrees);
        fixedArmS3Degrees = ClampFixedArmTarget(fixedArmS3Degrees);
        fixedCameraTiltDegrees = ClampFixedArmTarget(
            fixedCameraTiltDegrees);
        sensorPanActionDirection =
            sensorPanActionDirection < 0f ? -1f : 1f;
        fixedArmToleranceDegrees = Mathf.Clamp(
            fixedArmToleranceDegrees, 0.5f, 5f);
    }

    public void Configure(
        GfsxPhysicalTelemetry physicalTelemetry,
        GfsxRealVisionReceiver realVision)
    {
        telemetry = physicalTelemetry;
        vision = realVision;
    }

    public void ConfigureOutputPermissions(
        bool allowDrive,
        bool allowCameraPan,
        bool allowClaw)
    {
        allowDriveArming = allowDrive;
        allowCameraPanServo = allowCameraPan;
        allowClawServoActions = allowClaw;
    }

    public void ConfigurePhysicalPublishingSuppressed(bool suppressed)
    {
        suppressAllPhysicalPublishing = suppressed;
        if (suppressed)
        {
            publishersRegistered = false;
            EmergencyStop(
                "PHYSICAL PUBLISHING SUPPRESSED FOR EDITOR SMOKE TEST.");
        }
    }

    public void ConfigureLiveSafety(
        float linearLimit,
        float trackPwmLimit,
        float watchdogSeconds,
        bool requireRearIr)
    {
        maximumLinearCommand = Mathf.Clamp(linearLimit, 0.01f, 0.25f);
        maximumTrackPwm = Mathf.Clamp(trackPwmLimit, 35f, 100f);
        actionTimeoutSeconds = Mathf.Clamp(watchdogSeconds, 0.15f, 1f);
        requireFreshRearIrForReverse = requireRearIr;
    }

    public void ConfigureServoLimits(
        float panMinimumDegrees,
        float panMaximumDegrees,
        float panDegreesPerSecond,
        float openClawDegrees,
        float closedClawDegrees,
        float clawCooldownSeconds,
        float panActionDirection = 1f)
    {
        sensorPanMinimumDegrees = Mathf.Min(
            panMinimumDegrees, panMaximumDegrees);
        sensorPanMaximumDegrees = Mathf.Max(
            panMinimumDegrees, panMaximumDegrees);
        sensorPanDegreesPerSecond = Mathf.Max(1f, panDegreesPerSecond);
        sensorPanActionDirection = panActionDirection < 0f ? -1f : 1f;
        clawOpenDegrees = Mathf.Clamp(openClawDegrees, 0f, 180f);
        clawClosedDegrees = Mathf.Clamp(closedClawDegrees, 0f, 180f);
        clawCommandCooldownSeconds = Mathf.Clamp(
            clawCooldownSeconds, 0.1f, 3f);
    }

    public void ConfigureFixedArmPose(
        bool allow,
        bool requireBeforeDrive,
        float s1Degrees,
        float s2Degrees,
        float s3Degrees,
        float toleranceDegrees,
        float s6TiltDegrees = 75f)
    {
        allowFixedArmPose = allow;
        requireFixedArmPoseBeforeDrive = requireBeforeDrive;
        fixedArmS1Degrees = ClampFixedArmTarget(s1Degrees);
        fixedArmS2Degrees = ClampFixedArmTarget(s2Degrees);
        fixedArmS3Degrees = ClampFixedArmTarget(s3Degrees);
        fixedCameraTiltDegrees = ClampFixedArmTarget(s6TiltDegrees);
        fixedArmToleranceDegrees = Mathf.Clamp(
            toleranceDegrees, 0.5f, 5f);
    }

    public void AcceptPolicyAction(
        float gas,
        float steering,
        float cameraPan,
        int clawCommand)
    {
        AcceptPolicyAction(gas, steering, cameraPan, 1f, clawCommand);
    }

    public void AcceptPolicyAction(
        float gas,
        float steering,
        float cameraPan,
        float cameraPanSpeedMultiplier,
        int clawCommand)
    {
        latestGas = Sanitize(gas);
        latestSteering = Sanitize(steering);
        latestCameraPan = Sanitize(cameraPan);
        latestCameraPanSpeedMultiplier = Mathf.Clamp(
            float.IsNaN(cameraPanSpeedMultiplier) ||
            float.IsInfinity(cameraPanSpeedMultiplier)
                ? 1f
                : cameraPanSpeedMultiplier,
            1f,
            2f);
        latestClawCommand = Mathf.Clamp(clawCommand, 0, 2);
        lastActionTime = Time.realtimeSinceStartup;
        receivedActionCount++;
    }

    public void StopPolicyAction()
    {
        latestGas = 0f;
        latestSteering = 0f;
        latestCameraPan = 0f;
        latestCameraPanSpeedMultiplier = 1f;
        latestClawCommand = 0;
        ClearDriveMixer();
    }

    public void RequestDriveArm()
    {
        if (driveArmed)
        {
            EmergencyStop("Real drive manually disarmed.");
            return;
        }
        if (!ReadyForPhysicalDrive)
        {
            if (allowDriveArming && requireFixedArmPoseBeforeDrive &&
                (!fixedArmPoseArmed || !fixedArmPoseAtTarget))
            {
                safetyStatus =
                    "Cannot arm tracks: authorize fixed S1-S3 with F6 and wait for AT TARGET first.";
            }
            else
            {
                safetyStatus = allowDriveArming
                    ? "Cannot arm: ROS, sonar/IO1-IO4 sensors, vision, and a fresh policy action are all required."
                    : "Drive arming is blocked: the supplied ONNX training contract does not match the supplied P5 adapter and real odometry is unavailable.";
            }
            driveConfirmationDeadline = 0f;
            return;
        }
        float now = Time.realtimeSinceStartup;
        if (now <= driveConfirmationDeadline)
        {
            driveArmed = true;
            driveConfirmationDeadline = 0f;
            safetyStatus = "REAL DRIVE ARMED — F10/BACKSPACE stops immediately.";
        }
        else
        {
            driveConfirmationDeadline = now + ConfirmationWindowSeconds;
            safetyStatus = "ARM STEP 1/2: confirm robot is safely lifted/area clear, then press again within 8 s.";
        }
    }

    public void RequestCameraServoArm()
    {
        if (cameraServosRequested)
        {
            cameraServosRequested = false;
            cameraServosArmed = false;
            panTargetInitialized = false;
            servoConfirmationDeadline = 0f;
            servoConfirmationReadyAt = 0f;
            safetyStatus = clawServoRequested || fixedArmPoseRequested
                ? "Real S5 camera pan disabled; separately authorized servo channels remain active."
                : "Real S5 camera pan disabled.";
            if (!ServoGateRequested)
                PublishImmediateServoDisable();
            return;
        }
        if (!allowCameraPanServo || telemetry == null ||
            !telemetry.RosConnected || !telemetry.ServoStateFresh ||
            !telemetry.ServoArmedAckFresh)
        {
            safetyStatus = "Cannot arm S5: fresh six-servo Pi state/ack is required.";
            servoConfirmationDeadline = 0f;
            servoConfirmationReadyAt = 0f;
            return;
        }
        bool gateAlreadyRequested =
            clawServoRequested || fixedArmPoseRequested;
        if ((!gateAlreadyRequested && telemetry.ServoArmed) ||
            (gateAlreadyRequested && !telemetry.ServoArmed))
        {
            safetyStatus = gateAlreadyRequested
                ? "Cannot add S5: the existing servo gate has no armed Pi acknowledgement."
                : "Cannot arm S5: Pi reports a competing servo publisher already armed.";
            servoConfirmationDeadline = 0f;
            servoConfirmationReadyAt = 0f;
            return;
        }
        float now = Time.realtimeSinceStartup;
        if (now <= servoConfirmationDeadline)
        {
            if (now < servoConfirmationReadyAt)
            {
                safetyStatus =
                    $"S5 STEP 1/2: keep the Pi gate DISARMED for " +
                    $"{servoConfirmationReadyAt - now:F1} s, then confirm again.";
                return;
            }
            if (!telemetry.TryCopyServoState(servoTargets))
                return;
            panTargetDegrees = servoTargets[CameraPanServoIndex];
            panTargetInitialized = true;
            cameraServosRequested = true;
            cameraServosArmed = telemetry.ServoArmed;
            servoConfirmationDeadline = 0f;
            servoConfirmationReadyAt = 0f;
            safetyStatus = telemetry.ServoArmed
                ? "S5 camera pan authorized under the existing Pi servo gate."
                : "S5 enable requested. Waiting for Pi servo-gate acknowledgement.";
        }
        else
        {
            servoConfirmationDeadline = now + ConfirmationWindowSeconds;
            servoConfirmationReadyAt = gateAlreadyRequested
                ? now
                : now + MinimumServoDisarmDwellSeconds;
            if (!gateAlreadyRequested)
                PublishImmediateServoDisable();
            safetyStatus = gateAlreadyRequested
                ? "S5 STEP 1/2: existing authorized channels remain unchanged; confirm again within 8 s."
                : "S5 STEP 1/2: only camera pan will change; wait 1.1 s, then confirm within 8 s.";
        }
    }

    public void RequestClawServoArm()
    {
        if (clawServoRequested)
        {
            clawServoRequested = false;
            clawServoArmed = false;
            clawTargetInitialized = false;
            clawConfirmationDeadline = 0f;
            clawConfirmationReadyAt = 0f;
            safetyStatus = cameraServosRequested || fixedArmPoseRequested
                ? "Real S4 claw policy disabled; separately authorized servo channels remain active."
                : "Real S4 claw policy disabled.";
            if (!ServoGateRequested)
                PublishImmediateServoDisable();
            return;
        }
        if (!allowClawServoActions || telemetry == null ||
            !telemetry.RosConnected || !telemetry.ServoStateFresh ||
            !telemetry.ServoArmedAckFresh)
        {
            safetyStatus = allowClawServoActions
                ? "Cannot arm S4: fresh six-servo Pi state/ack is required."
                : "S4 policy output is not permitted in this scene.";
            clawConfirmationDeadline = 0f;
            clawConfirmationReadyAt = 0f;
            return;
        }
        if (!telemetry.TryCopyServoState(servoTargets))
            return;
        float currentClawDegrees = servoTargets[ClawServoIndex];
        float clawMinimumDegrees = Mathf.Min(
            clawOpenDegrees, clawClosedDegrees);
        float clawMaximumDegrees = Mathf.Max(
            clawOpenDegrees, clawClosedDegrees);
        if (currentClawDegrees < clawMinimumDegrees - 0.5f ||
            currentClawDegrees > clawMaximumDegrees + 0.5f)
        {
            safetyStatus =
                $"S4 ARM BLOCKED: Pi state {currentClawDegrees:F1} degrees is outside " +
                $"the calibrated {clawMinimumDegrees:F0}-{clawMaximumDegrees:F0} degree range. " +
                "Repair/calibrate S4 with tracks and policy outputs disabled first.";
            clawConfirmationDeadline = 0f;
            clawConfirmationReadyAt = 0f;
            return;
        }
        bool gateAlreadyRequested =
            cameraServosRequested || fixedArmPoseRequested;
        if ((!gateAlreadyRequested && telemetry.ServoArmed) ||
            (gateAlreadyRequested && !telemetry.ServoArmed))
        {
            safetyStatus = gateAlreadyRequested
                ? "Cannot add S4: the existing servo gate has no armed Pi acknowledgement."
                : "Cannot arm S4: Pi reports a competing servo publisher already armed.";
            clawConfirmationDeadline = 0f;
            clawConfirmationReadyAt = 0f;
            return;
        }
        float now = Time.realtimeSinceStartup;
        if (now <= clawConfirmationDeadline)
        {
            if (now < clawConfirmationReadyAt)
            {
                safetyStatus =
                    $"S4 STEP 1/2: keep the Pi gate DISARMED for " +
                    $"{clawConfirmationReadyAt - now:F1} s, then confirm again.";
                return;
            }
            if (!telemetry.TryCopyServoState(servoTargets))
                return;
            currentClawDegrees = servoTargets[ClawServoIndex];
            if (currentClawDegrees < clawMinimumDegrees - 0.5f ||
                currentClawDegrees > clawMaximumDegrees + 0.5f)
            {
                safetyStatus = "S4 changed outside its calibrated range; authorization cancelled.";
                clawConfirmationDeadline = 0f;
                clawConfirmationReadyAt = 0f;
                return;
            }
            clawTargetDegrees = currentClawDegrees;
            clawTargetInitialized = true;
            clawClosed = Mathf.Abs(
                currentClawDegrees - clawClosedDegrees) <= 0.5f;
            clawArmActionCount = receivedActionCount;
            lastProcessedClawActionCount = receivedActionCount;
            lastClawTargetChangeTime = now;
            clawServoRequested = true;
            clawServoArmed = telemetry.ServoArmed;
            clawConfirmationDeadline = 0f;
            clawConfirmationReadyAt = 0f;
            safetyStatus = telemetry.ServoArmed
                ? "S4 policy authorized under the existing Pi servo gate; NOOP preserves its current angle."
                : "S4 enable requested. Waiting for Pi servo-gate acknowledgement; NOOP preserves its current angle.";
        }
        else
        {
            clawConfirmationDeadline = now + ConfirmationWindowSeconds;
            clawConfirmationReadyAt = gateAlreadyRequested
                ? now
                : now + MinimumServoDisarmDwellSeconds;
            if (!gateAlreadyRequested)
                PublishImmediateServoDisable();
            safetyStatus = gateAlreadyRequested
                ? "S4 STEP 1/2: current calibrated angle will be preserved; confirm again within 8 s."
                : "S4 STEP 1/2: current calibrated angle will be preserved; wait 1.1 s, then confirm within 8 s.";
        }
    }

    public void RequestFixedArmPose()
    {
        if (fixedArmPoseRequested)
        {
            if (driveArmed)
            {
                EmergencyStop(
                    "Fixed S1-S3 pose was disabled while tracks were armed; all physical outputs stopped.");
                return;
            }

            fixedArmPoseRequested = false;
            fixedArmPoseArmed = false;
            fixedArmPoseAtTarget = false;
            fixedArmTargetsLatched = false;
            fixedArmConfirmationDeadline = 0f;
            fixedArmConfirmationReadyAt = 0f;
            safetyStatus = cameraServosRequested || clawServoRequested
                ? "Real fixed S1-S3 pose disabled; separately authorized servo channels remain active."
                : "Real fixed S1-S3 pose disabled; current arm command estimate is preserved.";
            if (!ServoGateRequested)
                PublishImmediateServoDisable();
            return;
        }

        if (driveArmed)
        {
            safetyStatus =
                "Cannot authorize S1-S3 while tracks are armed. Stop the tracks first.";
            fixedArmConfirmationDeadline = 0f;
            fixedArmConfirmationReadyAt = 0f;
            return;
        }

        if (!allowFixedArmPose || telemetry == null ||
            !telemetry.RosConnected || !telemetry.ServoStateFresh ||
            !telemetry.ServoArmedAckFresh)
        {
            safetyStatus = allowFixedArmPose
                ? "Cannot arm S1-S3: fresh six-servo Pi state/ack is required."
                : "Fixed S1-S3 physical pose is not permitted in this scene.";
            fixedArmConfirmationDeadline = 0f;
            fixedArmConfirmationReadyAt = 0f;
            return;
        }

        if (!FixedArmTargetsWithinLimits)
        {
            safetyStatus =
                $"S1-S3 ARM BLOCKED: every fixed target must be within " +
                $"{FixedArmMinimumDegrees:F0}-{FixedArmMaximumDegrees:F0} degrees.";
            fixedArmConfirmationDeadline = 0f;
            fixedArmConfirmationReadyAt = 0f;
            return;
        }

        if (!telemetry.TryCopyServoState(observedServoState))
            return;

        bool gateAlreadyRequested =
            cameraServosRequested || clawServoRequested;
        if ((!gateAlreadyRequested && telemetry.ServoArmed) ||
            (gateAlreadyRequested && !telemetry.ServoArmed))
        {
            safetyStatus = gateAlreadyRequested
                ? "Cannot add S1-S3: the existing S4/S5 servo gate has no armed Pi acknowledgement."
                : "Cannot arm S1-S3: Pi reports a competing servo publisher already armed.";
            fixedArmConfirmationDeadline = 0f;
            fixedArmConfirmationReadyAt = 0f;
            return;
        }

        float now = Time.realtimeSinceStartup;
        if (now <= fixedArmConfirmationDeadline)
        {
            if (now < fixedArmConfirmationReadyAt)
            {
                safetyStatus =
                    $"S1-S3 STEP 1/2: keep the Pi gate DISARMED for " +
                    $"{fixedArmConfirmationReadyAt - now:F1} s, then confirm again.";
                return;
            }

            if (!ValidFixedArmTarget(fixedArmCandidateS1Degrees) ||
                !ValidFixedArmTarget(fixedArmCandidateS2Degrees) ||
                !ValidFixedArmTarget(fixedArmCandidateS3Degrees) ||
                !ValidFixedArmTarget(fixedCameraTiltCandidateDegrees))
            {
                safetyStatus =
                    "S1-S3 candidate targets became invalid; authorization cancelled.";
                fixedArmConfirmationDeadline = 0f;
                fixedArmConfirmationReadyAt = 0f;
                return;
            }

            fixedArmLatchedS1Degrees = fixedArmCandidateS1Degrees;
            fixedArmLatchedS2Degrees = fixedArmCandidateS2Degrees;
            fixedArmLatchedS3Degrees = fixedArmCandidateS3Degrees;
            fixedCameraTiltLatchedDegrees =
                fixedCameraTiltCandidateDegrees;
            fixedArmTargetsLatched = true;
            fixedArmPoseRequested = true;
            fixedArmPoseArmed = telemetry.ServoArmed;
            fixedArmPoseAtTarget =
                fixedArmPoseArmed && FixedArmWithinTolerance(observedServoState);
            fixedArmConfirmationDeadline = 0f;
            fixedArmConfirmationReadyAt = 0f;
            safetyStatus = telemetry.ServoArmed
                ? "Fixed S1-S3 pose authorized under the existing Pi servo gate; tracks remain blocked until AT TARGET."
                : "Fixed S1-S3 pose requested. Waiting for Pi acknowledgement; tracks remain blocked.";
            return;
        }

        fixedArmCandidateS1Degrees = fixedArmS1Degrees;
        fixedArmCandidateS2Degrees = fixedArmS2Degrees;
        fixedArmCandidateS3Degrees = fixedArmS3Degrees;
        fixedCameraTiltCandidateDegrees = fixedCameraTiltDegrees;
        fixedArmConfirmationDeadline = now + ConfirmationWindowSeconds;
        fixedArmConfirmationReadyAt = gateAlreadyRequested
            ? now
            : now + MinimumServoDisarmDwellSeconds;
        if (!gateAlreadyRequested)
            PublishImmediateServoDisable();

        float deltaS1 = fixedArmCandidateS1Degrees -
            observedServoState[ShoulderServoIndex];
        float deltaS2 = fixedArmCandidateS2Degrees -
            observedServoState[ElbowServoIndex];
        float deltaS3 = fixedArmCandidateS3Degrees -
            observedServoState[WristServoIndex];
        float deltaS6 = fixedCameraTiltCandidateDegrees -
            observedServoState[CameraTiltServoIndex];
        string dwell = gateAlreadyRequested
            ? "confirm again within 8 s"
            : "wait 1.1 s, then confirm within 8 s";
        safetyStatus =
            $"S1-S3 STEP 1/2: target " +
            $"{fixedArmCandidateS1Degrees:F1}/{fixedArmCandidateS2Degrees:F1}/" +
            $"{fixedArmCandidateS3Degrees:F1} degrees, S6 " +
            $"{fixedCameraTiltCandidateDegrees:F1} degrees; moves " +
            $"{deltaS1:+0.0;-0.0;0.0}/{deltaS2:+0.0;-0.0;0.0}/" +
            $"{deltaS3:+0.0;-0.0;0.0}, S6 " +
            $"{deltaS6:+0.0;-0.0;0.0} degrees. Support and inspect the arm/camera, {dwell}.";
    }

    public void EmergencyStop(string reason = "EMERGENCY STOP")
    {
        driveArmed = false;
        cameraServosRequested = false;
        cameraServosArmed = false;
        clawServoRequested = false;
        clawServoArmed = false;
        fixedArmPoseRequested = false;
        fixedArmPoseArmed = false;
        fixedArmPoseAtTarget = false;
        driveConfirmationDeadline = 0f;
        servoConfirmationDeadline = 0f;
        servoConfirmationReadyAt = 0f;
        clawConfirmationDeadline = 0f;
        clawConfirmationReadyAt = 0f;
        fixedArmConfirmationDeadline = 0f;
        fixedArmConfirmationReadyAt = 0f;
        panTargetInitialized = false;
        clawTargetInitialized = false;
        fixedArmTargetsLatched = false;
        servoGateWasArmed = false;
        StopPolicyAction();
        safetyStatus = reason;
        PublishImmediateStop();
    }

    private void ResolveReferences()
    {
        if (telemetry == null)
            telemetry = GetComponent<GfsxPhysicalTelemetry>();
        if (vision == null)
            vision = GetComponent<GfsxRealVisionReceiver>();
    }

    private void ResetRuntimeState()
    {
        driveArmed = false;
        cameraServosRequested = false;
        cameraServosArmed = false;
        clawServoRequested = false;
        clawServoArmed = false;
        fixedArmPoseRequested = false;
        fixedArmPoseArmed = false;
        fixedArmPoseAtTarget = false;
        publishersRegistered = false;
        nextPublishTime = 0f;
        receivedActionCount = 0;
        lastActionTime = float.NegativeInfinity;
        panTargetInitialized = false;
        clawTargetInitialized = false;
        clawTargetDegrees = 0f;
        clawClosed = false;
        driveConfirmationDeadline = 0f;
        servoConfirmationDeadline = 0f;
        servoConfirmationReadyAt = 0f;
        clawConfirmationDeadline = 0f;
        clawConfirmationReadyAt = 0f;
        fixedArmConfirmationDeadline = 0f;
        fixedArmConfirmationReadyAt = 0f;
        fixedArmCandidateS1Degrees = 0f;
        fixedArmCandidateS2Degrees = 0f;
        fixedArmCandidateS3Degrees = 0f;
        fixedCameraTiltCandidateDegrees = 0f;
        fixedArmLatchedS1Degrees = 0f;
        fixedArmLatchedS2Degrees = 0f;
        fixedArmLatchedS3Degrees = 0f;
        fixedCameraTiltLatchedDegrees = 0f;
        fixedArmTargetsLatched = false;
        clawArmActionCount = 0;
        lastProcessedClawActionCount = 0;
        lastClawTargetChangeTime = float.NegativeInfinity;
        servoGateWasArmed = false;
        forwardOverrideActive = false;
        reverseOverrideActive = false;
        safetyStatus = "DRY RUN: model decisions are visible; all real outputs are DISARMED.";
        StopPolicyAction();
    }

    private void ReadEmergencyAndArmKeys()
    {
        bool driveKey;
        bool servoKey;
        bool clawKey;
        bool fixedArmKey;
        bool emergencyKey;
#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;
        driveKey = keyboard != null && keyboard.f9Key.wasPressedThisFrame;
        servoKey = keyboard != null && keyboard.f8Key.wasPressedThisFrame;
        clawKey = keyboard != null && keyboard.f7Key.wasPressedThisFrame;
        fixedArmKey = keyboard != null && keyboard.f6Key.wasPressedThisFrame;
        emergencyKey = keyboard != null &&
            (keyboard.f10Key.wasPressedThisFrame ||
             keyboard.backspaceKey.wasPressedThisFrame);
#else
        driveKey = Input.GetKeyDown(KeyCode.F9);
        servoKey = Input.GetKeyDown(KeyCode.F8);
        clawKey = Input.GetKeyDown(KeyCode.F7);
        fixedArmKey = Input.GetKeyDown(KeyCode.F6);
        emergencyKey = Input.GetKeyDown(KeyCode.F10) ||
            Input.GetKeyDown(KeyCode.Backspace);
#endif
        if (emergencyKey)
            EmergencyStop();
        else
        {
            if (driveKey)
                RequestDriveArm();
            if (servoKey)
                RequestCameraServoArm();
            if (clawKey)
                RequestClawServoArm();
            if (fixedArmKey)
                RequestFixedArmPose();
        }
    }

    private void RefreshSafetyState()
    {
        if (driveArmed && telemetry != null && telemetry.MotorPwmFresh &&
            (Mathf.Abs(telemetry.LeftPwm) > maximumTrackPwm + 0.5f ||
             Mathf.Abs(telemetry.RightPwm) > maximumTrackPwm + 0.5f))
        {
            EmergencyStop(
                $"FAIL-CLOSED: Pi applied PWM {telemetry.LeftPwm:F1}/" +
                $"{telemetry.RightPwm:F1}, above the {maximumTrackPwm:F0} limit.");
            return;
        }
        if (driveArmed && !ReadyForPhysicalDrive)
        {
            EmergencyStop(
                requireFixedArmPoseBeforeDrive &&
                (!fixedArmPoseArmed || !fixedArmPoseAtTarget)
                    ? "FAIL-CLOSED: fixed S1-S3 pose is no longer armed and AT TARGET."
                    : "FAIL-CLOSED: physical telemetry, vision, ROS, or policy action became stale.");
            return;
        }
        if (ServoGateRequested &&
            (telemetry == null || !telemetry.RosConnected ||
             !telemetry.ServoStateFresh || !telemetry.ServoArmedAckFresh))
        {
            EmergencyStop("FAIL-CLOSED: servo state/ack became stale.");
            return;
        }
        if (!ServoGateRequested)
        {
            cameraServosArmed = false;
            clawServoArmed = false;
            fixedArmPoseArmed = false;
            fixedArmPoseAtTarget = false;
            servoGateWasArmed = false;
            return;
        }
        if (telemetry.ServoArmed)
        {
            servoGateWasArmed = true;
            cameraServosArmed = cameraServosRequested;
            clawServoArmed = clawServoRequested;
            fixedArmPoseArmed = fixedArmPoseRequested;
            fixedArmPoseAtTarget = fixedArmPoseArmed &&
                fixedArmTargetsLatched &&
                telemetry.TryCopyServoState(observedServoState) &&
                FixedArmWithinTolerance(observedServoState);
            safetyStatus = BuildServoGateStatus();
        }
        else
        {
            cameraServosArmed = false;
            clawServoArmed = false;
            fixedArmPoseArmed = false;
            fixedArmPoseAtTarget = false;
            if (servoGateWasArmed)
                EmergencyStop("FAIL-CLOSED: Pi servo gate acknowledgement dropped.");
        }
    }

    private void UpdateDriveMixer()
    {
        bool commandUsable = HasFreshPolicyAction;
        float gas = commandUsable ? latestGas : 0f;
        float steering = commandUsable ? latestSteering : 0f;
        forwardOverrideActive = telemetry != null && gas > 0f &&
            (telemetry.UltrasonicMetres <= forwardStopDistanceMetres ||
             (stopForwardOnFrontIr &&
              (telemetry.LeftIr > 0.5f || telemetry.RightIr > 0.5f)));
        reverseOverrideActive = driveArmed && gas < 0f &&
            requireFreshRearIrForReverse &&
            (telemetry == null || !telemetry.RearIrFresh ||
             telemetry.RearIr > 0.5f);
        if (forwardOverrideActive || reverseOverrideActive)
            gas = 0f;

        float requestedLinear = Mathf.Clamp(
            gas * moveSpeed,
            -maximumLinearCommand,
            maximumLinearCommand);
        float requestedTurn = steering * turnContribution;
        float requestedLeftPwm = SpeedToPwm(requestedLinear + requestedTurn);
        float requestedRightPwm = SpeedToPwm(requestedLinear - requestedTurn);
        leftPwm = Mathf.MoveTowards(leftPwm, requestedLeftPwm, maximumPwmStep);
        rightPwm = Mathf.MoveTowards(rightPwm, requestedRightPwm, maximumPwmStep);
        float leftSpeed = leftPwm / pwmPerMetrePerSecond;
        float rightSpeed = rightPwm / pwmPerMetrePerSecond;
        mixedLinearSpeed = Mathf.Clamp(
            (leftSpeed + rightSpeed) * 0.5f,
            -maximumLinearCommand,
            maximumLinearCommand);
        // The Pi reconstructs wheel speeds as linear +/- angular * TURN_K.
        // Encode the limited pair with that same gain instead of simulator
        // yaw geometry, otherwise a nominal 35 PWM pivot becomes ~61 PWM.
        mixedYawRateDegrees =
            (leftSpeed - rightSpeed) /
            (2f * PhysicalBridgeTurnGain) * Mathf.Rad2Deg;
        if (Mathf.Approximately(gas, 0f) && Mathf.Approximately(steering, 0f))
            ClearDriveMixer();
    }

    private void UpdateServoTargets()
    {
        if (!ServoGateRequested || telemetry == null ||
            !telemetry.TryCopyServoState(servoTargets))
            return;
        if (fixedArmPoseRequested && fixedArmTargetsLatched)
        {
            servoTargets[ShoulderServoIndex] = fixedArmLatchedS1Degrees;
            servoTargets[ElbowServoIndex] = fixedArmLatchedS2Degrees;
            servoTargets[WristServoIndex] = fixedArmLatchedS3Degrees;
            servoTargets[CameraTiltServoIndex] =
                fixedCameraTiltLatchedDegrees;
        }
        if (cameraServosRequested && !panTargetInitialized)
        {
            panTargetDegrees = servoTargets[CameraPanServoIndex];
            panTargetInitialized = true;
        }
        if (cameraServosRequested)
        {
            panTargetDegrees = Mathf.Clamp(
                panTargetDegrees + latestCameraPan *
                latestCameraPanSpeedMultiplier *
                sensorPanActionDirection * sensorPanDegreesPerSecond *
                Time.fixedDeltaTime,
                sensorPanMinimumDegrees,
                sensorPanMaximumDegrees);
            servoTargets[CameraPanServoIndex] = panTargetDegrees;
        }

        if (clawServoRequested && clawTargetInitialized)
        {
            if (receivedActionCount > clawArmActionCount &&
                receivedActionCount > lastProcessedClawActionCount)
            {
                lastProcessedClawActionCount = receivedActionCount;
                float now = Time.realtimeSinceStartup;
                if (now - lastClawTargetChangeTime >=
                    clawCommandCooldownSeconds)
                {
                    if (latestClawCommand == 1 &&
                        Mathf.Abs(clawTargetDegrees - clawClosedDegrees) > 0.5f)
                    {
                        clawClosed = true;
                        clawTargetDegrees = clawClosedDegrees;
                        lastClawTargetChangeTime = now;
                    }
                    else if (latestClawCommand == 2 &&
                        Mathf.Abs(clawTargetDegrees - clawOpenDegrees) > 0.5f)
                    {
                        clawClosed = false;
                        clawTargetDegrees = clawOpenDegrees;
                        lastClawTargetChangeTime = now;
                    }
                }
            }
            servoTargets[ClawServoIndex] = clawTargetDegrees;
        }
    }

    private void PublishHeartbeatAndCommands()
    {
        if (!EnsurePublishers())
            return;
        bool sendDrive = driveArmed && ReadyForPhysicalDrive;
        ros.Publish(DriveEnableTopic, new BoolMsg(sendDrive));
        ros.Publish(
            CmdVelTopic,
            sendDrive
                ? new TwistMsg(
                    new Vector3Msg(mixedLinearSpeed, 0d, 0d),
                    // Preserve the direction already validated by P2 manual control.
                    new Vector3Msg(0d, 0d, -mixedYawRateDegrees * Mathf.Deg2Rad))
                : new TwistMsg(new Vector3Msg(), new Vector3Msg()));

        bool sendServoGate = ServoGateRequested && telemetry != null &&
            telemetry.TryCopyServoState(servoTargets);
        if (sendServoGate)
        {
            // Refresh the complete Pi baseline on every publish and replace
            // only separately authorized channels. S6 and every unauthorized
            // arm/claw/camera channel always pass through byte-for-byte.
            if (fixedArmPoseRequested && fixedArmTargetsLatched)
            {
                servoTargets[ShoulderServoIndex] = fixedArmLatchedS1Degrees;
                servoTargets[ElbowServoIndex] = fixedArmLatchedS2Degrees;
                servoTargets[WristServoIndex] = fixedArmLatchedS3Degrees;
                servoTargets[CameraTiltServoIndex] =
                    fixedCameraTiltLatchedDegrees;
            }
            if (cameraServosRequested && panTargetInitialized)
                servoTargets[CameraPanServoIndex] = panTargetDegrees;
            if (clawServoRequested && clawTargetInitialized)
                servoTargets[ClawServoIndex] = clawTargetDegrees;
            ros.Publish(ServoTopic, new Float32MultiArrayMsg
            {
                data = (float[])servoTargets.Clone()
            });
        }
        ros.Publish(ServoEnableTopic, new BoolMsg(sendServoGate));
    }

    private bool EnsurePublishers()
    {
        if (suppressAllPhysicalPublishing || ros == null)
            return false;
        try
        {
            if (!IsPublisher(CmdVelTopic))
                ros.RegisterPublisher<TwistMsg>(CmdVelTopic, 2);
            if (!IsPublisher(DriveEnableTopic))
                ros.RegisterPublisher<BoolMsg>(DriveEnableTopic, 2);
            if (!IsPublisher(ServoTopic))
                ros.RegisterPublisher<Float32MultiArrayMsg>(ServoTopic, 2);
            if (!IsPublisher(ServoEnableTopic))
                ros.RegisterPublisher<BoolMsg>(ServoEnableTopic, 2);
            publishersRegistered = IsPublisher(CmdVelTopic) &&
                IsPublisher(DriveEnableTopic) && IsPublisher(ServoTopic) &&
                IsPublisher(ServoEnableTopic);
        }
        catch (System.Exception)
        {
            publishersRegistered = false;
        }
        return publishersRegistered;
    }

    private bool IsPublisher(string topic)
    {
        return ros != null && ros.GetTopic(topic)?.IsPublisher == true;
    }

    private void PublishImmediateStop()
    {
        if (!publishersRegistered || ros == null || !IsPublisher(CmdVelTopic) ||
            !IsPublisher(DriveEnableTopic) || !IsPublisher(ServoEnableTopic))
            return;
        ros.Publish(DriveEnableTopic, new BoolMsg(false));
        ros.Publish(ServoEnableTopic, new BoolMsg(false));
        ros.Publish(
            CmdVelTopic,
            new TwistMsg(new Vector3Msg(), new Vector3Msg()));
    }

    private void PublishImmediateServoDisable()
    {
        if (!publishersRegistered || ros == null ||
            !IsPublisher(ServoEnableTopic))
            return;
        ros.Publish(ServoEnableTopic, new BoolMsg(false));
    }

    private bool FixedArmWithinTolerance(float[] state)
    {
        return state != null && state.Length == ServoCount &&
            fixedArmTargetsLatched &&
            Mathf.Abs(state[ShoulderServoIndex] -
                      fixedArmLatchedS1Degrees) <= fixedArmToleranceDegrees &&
            Mathf.Abs(state[ElbowServoIndex] -
                      fixedArmLatchedS2Degrees) <= fixedArmToleranceDegrees &&
            Mathf.Abs(state[WristServoIndex] -
                      fixedArmLatchedS3Degrees) <= fixedArmToleranceDegrees &&
            Mathf.Abs(state[CameraTiltServoIndex] -
                      fixedCameraTiltLatchedDegrees) <=
                fixedArmToleranceDegrees;
    }

    private string BuildServoGateStatus()
    {
        string arm = fixedArmPoseArmed
            ? fixedArmPoseAtTarget
                ? "S1-S3/S6 fixed pose AT TARGET"
                : "S1-S3/S6 fixed pose MOVING (Pi command estimate)"
            : "S1-S3/S6 unchanged";
        string claw = clawServoArmed
            ? "S4 policy ARMED"
            : "S4 unchanged";
        string camera = cameraServosArmed
            ? "S5 pan ARMED"
            : "S5 unchanged";
        return $"Pi servo gate ARMED: {arm}; {claw}; {camera}.";
    }

    private static bool ValidFixedArmTarget(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value) &&
            value >= FixedArmMinimumDegrees &&
            value <= FixedArmMaximumDegrees;
    }

    private static float ClampFixedArmTarget(float value)
    {
        if (float.IsNaN(value) || float.IsInfinity(value))
            return 90f;
        return Mathf.Clamp(
            value, FixedArmMinimumDegrees, FixedArmMaximumDegrees);
    }

    private float SpeedToPwm(float speed)
    {
        float pwm = Mathf.Clamp(
            speed * pwmPerMetrePerSecond,
            -maximumTrackPwm,
            maximumTrackPwm);
        float magnitude = Mathf.Abs(pwm);
        if (magnitude <= motorDeadZone)
            return 0f;
        return Mathf.Sign(pwm) * Mathf.Min(
            Mathf.Max(magnitude, minimumMovingPwm),
            maximumTrackPwm);
    }

    private void ClearDriveMixer()
    {
        leftPwm = 0f;
        rightPwm = 0f;
        mixedLinearSpeed = 0f;
        mixedYawRateDegrees = 0f;
    }

    private static float Sanitize(float value)
    {
        return float.IsNaN(value) || float.IsInfinity(value)
            ? 0f
            : Mathf.Clamp(value, -1f, 1f);
    }

    private void OnGUI()
    {
        const float width = 520f;
        float height = Mathf.Min(560f, Screen.height - 24f);
        Rect panel = new Rect(12f, 12f, width, height);
        GUI.Box(panel, "GFS-X PHYSICAL AGENT • REAL CAMERA + REAL SENSORS");
        GUILayout.BeginArea(new Rect(panel.x + 12f, panel.y + 28f, width - 24f, height - 38f));
        hudScroll = GUILayout.BeginScrollView(hudScroll);
        GUILayout.Label(
            $"ROS: {(telemetry != null && telemetry.RosConnected ? "CONNECTED" : "OFFLINE")}  " +
            $"sensors: {(telemetry != null && telemetry.SensorFresh ? "FRESH" : "STALE")}  " +
            $"vision: {(vision != null && vision.HasFreshPacket ? "FRESH" : "STALE")}");
        if (telemetry != null)
        {
            GUILayout.Label(
                $"sonar {telemetry.UltrasonicMetres:F2} m | IR IO2-left/IO1-right/IO3-claw/IO4-rear " +
                $"{telemetry.LeftIr:F0}/{telemetry.RightIr:F0}/{telemetry.GripperIr:F0}/{telemetry.RearIr:F0}");
            GUILayout.Label(
                $"estimated odometry x/z {telemetry.EstimatedDisplacement.x:F2}/" +
                $"{telemetry.EstimatedDisplacement.y:F2} m, heading {telemetry.EstimatedHeadingDegrees:F1} degrees");
        }
        if (vision != null)
        {
            GUILayout.Label(
                vision.BallVisible
                    ? $"BALL: angle {vision.NormalizedAngle:+0.00;-0.00;0.00}, " +
                      $"distance {vision.NormalizedDistance:F2} ({vision.DistanceMetres:F2} m), " +
                      $"confidence {vision.Confidence:F2}"
                    : "BALL: not detected");
        }
        GUILayout.Label(
            $"policy #{receivedActionCount}: gas {latestGas:+0.00;-0.00;0.00}, " +
            $"steer {latestSteering:+0.00;-0.00;0.00}, pan {latestCameraPan:+0.00;-0.00;0.00}, " +
            $"claw {latestClawCommand} " +
            $"{(clawServoArmed ? "ARMED" : allowClawServoActions ? "AVAILABLE" : "BLOCKED")}");
        if (telemetry != null &&
            telemetry.TryCopyServoState(observedServoState))
        {
            GUILayout.Label(
                $"S1/S2/S3 Pi estimate " +
                $"{observedServoState[ShoulderServoIndex]:F1}/" +
                $"{observedServoState[ElbowServoIndex]:F1}/" +
                $"{observedServoState[WristServoIndex]:F1} deg | " +
                $"configured {fixedArmS1Degrees:F1}/" +
                $"{fixedArmS2Degrees:F1}/{fixedArmS3Degrees:F1}");
        }
        if (!allowDriveArming)
            GUILayout.Label("SHADOW MODE LOCK: real track arming is disabled by preflight contract audit.");
        if (forwardOverrideActive || reverseOverrideActive)
            GUILayout.Label(
                reverseOverrideActive
                    ? "SAFETY OVERRIDE: reverse blocked by stale/active IO4 rear IR."
                    : "SAFETY OVERRIDE: forward motion blocked by IO1/IO2/sonar.");

        Color original = GUI.backgroundColor;
        GUI.backgroundColor = driveArmed ? Color.red : new Color(0.55f, 0.9f, 0.55f);
        if (GUILayout.Button(
            driveArmed
                ? "DISARM REAL TRACKS (F9 / fn-F9)"
                : Time.realtimeSinceStartup <= driveConfirmationDeadline
                    ? "CONFIRM TRACKS ARE SAFE (STEP 2/2)"
                    : "PREPARE REAL TRACKS — LIFTED TEST FIRST (STEP 1/2)",
            GUILayout.Height(36f)))
        {
            RequestDriveArm();
        }

        GUI.backgroundColor = fixedArmPoseRequested
            ? new Color(1f, 0.65f, 0.25f)
            : Color.white;
        if (GUILayout.Button(
            fixedArmPoseRequested
                ? "DISABLE REAL S1-S3 FIXED POSE (F6 / fn-F6)"
                : Time.realtimeSinceStartup <= fixedArmConfirmationDeadline
                    ? "CONFIRM REAL S1-S3 FIXED POSE (STEP 2/2)"
                    : "PREPARE REAL S1-S3 FIXED POSE (STEP 1/2)",
            GUILayout.Height(32f)))
        {
            RequestFixedArmPose();
        }

        GUI.backgroundColor = cameraServosRequested
            ? new Color(1f, 0.65f, 0.25f)
            : Color.white;
        if (GUILayout.Button(
            cameraServosRequested
                ? "DISABLE REAL S5 CAMERA PAN (F8 / fn-F8)"
                : Time.realtimeSinceStartup <= servoConfirmationDeadline
                    ? "CONFIRM REAL S5 ONLY (STEP 2/2)"
                    : "PREPARE REAL S5 CAMERA PAN (STEP 1/2)",
            GUILayout.Height(32f)))
        {
            RequestCameraServoArm();
        }

        GUI.backgroundColor = clawServoRequested
            ? new Color(1f, 0.65f, 0.25f)
            : Color.white;
        if (GUILayout.Button(
            clawServoRequested
                ? "DISABLE REAL S4 CLAW POLICY (F7 / fn-F7)"
                : Time.realtimeSinceStartup <= clawConfirmationDeadline
                    ? "CONFIRM REAL S4 CLAW (STEP 2/2)"
                    : "PREPARE REAL S4 CLAW — 0 OPEN / 50 CLOSED (STEP 1/2)",
            GUILayout.Height(32f)))
        {
            RequestClawServoArm();
        }

        GUI.backgroundColor = Color.red;
        if (GUILayout.Button("EMERGENCY STOP (F10 / BACKSPACE)", GUILayout.Height(42f)))
            EmergencyStop();
        GUI.backgroundColor = original;

        GUILayout.Label("Status: " + safetyStatus);
        GUILayout.Label(
            "F6 fixes S1-S3 independently; its AT TARGET state is the Pi command " +
            "estimate, not encoder feedback. F8 controls S5 and F7 controls S4. " +
            "S4: 0 degrees open, 50 degrees closed; action 0 preserves its target. " +
            "S6 always remains at the exact fresh Pi value.");
        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }
}
