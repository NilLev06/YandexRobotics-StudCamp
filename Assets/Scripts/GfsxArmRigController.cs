using System;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif
using UnityEngine.Serialization;

[ExecuteAlways]
[DisallowMultipleComponent]
public sealed class GfsxArmRigController : MonoBehaviour
{
    public const float DefaultS4MaximumClosureCommand = 22f;
    private static readonly string[] BinaryJawLabels =
        { "OPEN", "CLOSED" };

    public enum S4JawControlMode
    {
        Continuous,
        BinaryOpenClosed
    }

    [Obsolete(
        "This is only the legacy default. Read S4MaximumClosureLimit from " +
        "the controller instance for the configured closing limit.")]
    public const float S4MaximumClosureCommand =
        DefaultS4MaximumClosureCommand;

    [Serializable]
    private struct LockedPose
    {
        public Transform target;
        public Vector3 localPosition;
        public Quaternion localRotation;
        public Vector3 localScale;
    }

    [Header("Servo and rigid-link references")]
    [SerializeField] private Transform s1ShoulderPivot;
    [SerializeField, FormerlySerializedAs("s2ElbowPivot")]
    private Transform rigidArmLink;
    [SerializeField, FormerlySerializedAs("s3WristPivot")]
    private Transform s2ElbowPivot;
    [SerializeField] private Transform s3WristRollPivot;
    [SerializeField] private Transform upperJawPivot;
    [SerializeField] private Transform lowerJawPivot;
    [SerializeField] private Transform upperJaw;
    [SerializeField] private Transform lowerJaw;
    [SerializeField] private Transform holdPoint;
    [SerializeField] private Transform gripperIrPoint;

    [Header("Servo commands (degrees from the imported neutral pose)")]
    [SerializeField] private float s1Angle;
    [SerializeField, FormerlySerializedAs("s3Angle")]
    private float s2Angle;
    [SerializeField] private float s3WristRollAngle;
    [SerializeField] private float s4Closure;

    [Header("Allowed servo angles (editable in Unity)")]
    [SerializeField, Range(-180f, 0f)] private float s1MinimumAngle = -55f;
    [SerializeField, Range(0f, 180f)] private float s1MaximumAngle = 55f;
    [SerializeField, Range(-180f, 0f)] private float s2MinimumAngle = -75f;
    [SerializeField, Range(0f, 180f)] private float s2MaximumAngle = 75f;
    [SerializeField, Range(-180f, 0f)] private float s3MinimumAngle = -90f;
    [SerializeField, Range(0f, 180f)] private float s3MaximumAngle = 90f;

    [Header("S4 servo control and positions (degrees)")]
    [SerializeField, InspectorName("Control Mode")]
    [Tooltip(
        "Binary Open/Closed stores only the two configured S4 servo " +
        "positions. Continuous allows every position between them.")]
    private S4JawControlMode s4ControlMode =
        S4JawControlMode.BinaryOpenClosed;
    [SerializeField, Range(0f, 179f),
     InspectorName("Open S4 Servo Position (degrees)")]
    [Tooltip("Real S4 servo target used for the fully open state.")]
    private float s4MinimumClosureCommand;
    [SerializeField, Range(1f, 180f),
     InspectorName("Closed S4 Servo Position (degrees)")]
    [Tooltip("Real S4 servo target used for the fully closed state.")]
    private float s4MaximumClosureLimit = DefaultS4MaximumClosureCommand;

    [Header("Jaw mesh geometry (each finger, degrees)")]
    [SerializeField, Range(0f, 15f),
     InspectorName("Closed Jaw Hinge Angle (degrees)")]
    private float closedJawOpeningAngle = 4f;
    [SerializeField, Range(20f, 45f),
     InspectorName("Open Jaw Hinge Angle (degrees)")]
    private float openJawOpeningAngle = 40f;

    [Header("Keyboard")]
    [SerializeField] private bool readKeyboard = true;
    [SerializeField] private float armDegreesPerSecond = 35f;
    [SerializeField] private bool showControlsOverlay = true;

    [Header("Episode start pose (straight arm, S1 sets height)")]
    [Tooltip("S1 shoulder — only joint lowered for floor pickup.")]
    [SerializeField] private float floorPickupS1 = -38f;
    [Tooltip("S2 elbow — 0° keeps the arm strictly straight.")]
    [SerializeField] private float floorPickupS2 = 0f;
    [SerializeField] private float floorPickupS3 = 90f;

    [Header("Floor clearance (no ground contact)")]
    [Tooltip("Claw tip must stay strictly above the floor by this margin (metres). Near-zero, but non-contact.")]
    [SerializeField, Min(0.0005f)] private float minTipClearanceMetres = 0.003f;
    [Tooltip("How far above floor-pickup S1 the clearance fixer may raise the shoulder.")]
    [SerializeField, Range(0f, 40f)] private float maxClearanceRaiseDegrees = 12f;
    [SerializeField] private LayerMask floorRaycastLayers = ~0;

    private readonly RaycastHit[] floorHitBuffer = new RaycastHit[16];

    public bool KeyboardControlEnabled
    {
        get => readKeyboard;
        set => readKeyboard = value;
    }

    [Header("Calibrated servo axes")]
    [SerializeField] private Vector3 s1LocalAxis = Vector3.forward;
    [SerializeField, FormerlySerializedAs("s3LocalAxis")]
    private Vector3 s2LocalAxis = Vector3.forward;
    [SerializeField] private Vector3 s3WristRollLocalAxis = Vector3.right;
    [SerializeField, FormerlySerializedAs("jawLocalAxis")]
    private Vector3 jawPivotLocalAxis = Vector3.up;
    [SerializeField] private float upperJawSign = 1f;
    [SerializeField] private float lowerJawSign = -1f;

    [Header("Captured mechanical bind pose")]
    [SerializeField, HideInInspector] private bool bindPoseCaptured;
    [SerializeField, HideInInspector] private Vector3 s1LocalPosition;
    [SerializeField, HideInInspector] private Quaternion s1NeutralRotation;
    [SerializeField, HideInInspector] private Vector3 s1LocalScale;
    [SerializeField, HideInInspector, FormerlySerializedAs("s2LocalPosition")]
    private Vector3 rigidLinkLocalPosition;
    [SerializeField, HideInInspector, FormerlySerializedAs("s2NeutralRotation")]
    private Quaternion rigidLinkNeutralRotation;
    [SerializeField, HideInInspector, FormerlySerializedAs("s2LocalScale")]
    private Vector3 rigidLinkLocalScale;
    [SerializeField, HideInInspector, FormerlySerializedAs("s3LocalPosition")]
    private Vector3 s2LocalPosition;
    [SerializeField, HideInInspector, FormerlySerializedAs("s3NeutralRotation")]
    private Quaternion s2NeutralRotation;
    [SerializeField, HideInInspector, FormerlySerializedAs("s3LocalScale")]
    private Vector3 s2LocalScale;
    [SerializeField, HideInInspector] private Vector3 s3WristRollLocalPosition;
    [SerializeField, HideInInspector] private Quaternion s3WristRollNeutralRotation;
    [SerializeField, HideInInspector] private Vector3 s3WristRollLocalScale;
    [SerializeField, HideInInspector] private Vector3 upperJawPivotLocalPosition;
    [SerializeField, HideInInspector] private Quaternion upperJawPivotNeutralRotation;
    [SerializeField, HideInInspector] private Vector3 upperJawPivotLocalScale;
    [SerializeField, HideInInspector] private Vector3 lowerJawPivotLocalPosition;
    [SerializeField, HideInInspector] private Quaternion lowerJawPivotNeutralRotation;
    [SerializeField, HideInInspector] private Vector3 lowerJawPivotLocalScale;
    [SerializeField, HideInInspector] private Vector3 upperJawLocalPosition;
    [SerializeField, HideInInspector] private Quaternion upperJawNeutralRotation;
    [SerializeField, HideInInspector] private Vector3 upperJawLocalScale;
    [SerializeField, HideInInspector] private Vector3 lowerJawLocalPosition;
    [SerializeField, HideInInspector] private Quaternion lowerJawNeutralRotation;
    [SerializeField, HideInInspector] private Vector3 lowerJawLocalScale;
    [SerializeField, HideInInspector] private LockedPose[] lockedVisualPoses = Array.Empty<LockedPose>();

    public bool IsConfigured =>
        bindPoseCaptured &&
        s1ShoulderPivot != null &&
        rigidArmLink != null &&
        s2ElbowPivot != null &&
        s3WristRollPivot != null &&
        upperJawPivot != null &&
        lowerJawPivot != null &&
        upperJaw != null &&
        lowerJaw != null;

    public float S1Angle => s1Angle;
    public float S2Angle => s2Angle;
    public float S3Angle => s3WristRollAngle;
    public float S4Closure => s4Closure;
    public float S1MinimumAngle => s1MinimumAngle;
    public float S1MaximumAngle => s1MaximumAngle;
    public float S2MinimumAngle => s2MinimumAngle;
    public float S2MaximumAngle => s2MaximumAngle;
    public float S3MinimumAngle => s3MinimumAngle;
    public float S3MaximumAngle => s3MaximumAngle;
    public float S4MinimumClosureLimit => s4MinimumClosureCommand;
    public float S4MaximumClosureLimit => s4MaximumClosureLimit;
    public S4JawControlMode S4ControlMode => s4ControlMode;
    public bool IsS4Binary =>
        s4ControlMode == S4JawControlMode.BinaryOpenClosed;
    public float S4OpenPositionDegrees => s4MinimumClosureCommand;
    public float S4ClosedPositionDegrees => s4MaximumClosureLimit;
    public float S4NormalizedClosure => Mathf.InverseLerp(
        s4MinimumClosureCommand,
        s4MaximumClosureLimit,
        s4Closure);
    public bool IsJawClosed => S4NormalizedClosure >= 0.5f;
    public Transform HoldPoint => holdPoint;
    public Transform GripperIrPoint => gripperIrPoint;
    public float FloorPickupS1 => floorPickupS1;
    public float FloorPickupS2 => floorPickupS2;
    public float FloorPickupS3 => floorPickupS3;
    public float MinTipClearanceMetres => minTipClearanceMetres;
    public float ClawTipClearanceAboveFloor => GetClawClearanceAboveFloor();
    /// <summary>Highest S1 the floor-clearance helper is allowed to command.</summary>
    public float MaxS1ForFloorClearance =>
        Mathf.Min(s1MaximumAngle, floorPickupS1 + maxClearanceRaiseDegrees);

    private void Update()
    {
        if (Application.isPlaying && readKeyboard)
            ReadKeyboard();

        ApplyPose();
    }

    private void LateUpdate()
    {
        // Re-apply after every other component. This prevents individual arm
        // parts from being translated, scaled, or detached by runtime logic.
        ApplyPose();
        if (Application.isPlaying)
            EnforceFloorClearance();
    }

    private void OnValidate()
    {
        ClampCommands();
        ApplyPose();
    }

    private void ReadKeyboard()
    {
        bool s1Negative;
        bool s1Positive;
        bool s2Negative;
        bool s2Positive;
        bool s3Negative;
        bool s3Positive;
        bool s1NegativePressed;
        bool s1PositivePressed;
        bool s2NegativePressed;
        bool s2PositivePressed;
        bool s3NegativePressed;
        bool s3PositivePressed;
        bool closePressed;
        bool openPressed;
        bool resetPressed;

#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;
        if (keyboard == null)
            return;

        s1Negative = keyboard.digit1Key.isPressed;
        s1Positive = keyboard.digit2Key.isPressed;
        s2Negative = keyboard.digit3Key.isPressed;
        s2Positive = keyboard.digit4Key.isPressed;
        s3Negative = keyboard.digit5Key.isPressed;
        s3Positive = keyboard.digit6Key.isPressed;
        s1NegativePressed = keyboard.digit1Key.wasPressedThisFrame;
        s1PositivePressed = keyboard.digit2Key.wasPressedThisFrame;
        s2NegativePressed = keyboard.digit3Key.wasPressedThisFrame;
        s2PositivePressed = keyboard.digit4Key.wasPressedThisFrame;
        s3NegativePressed = keyboard.digit5Key.wasPressedThisFrame;
        s3PositivePressed = keyboard.digit6Key.wasPressedThisFrame;
        closePressed = keyboard.digit7Key.wasPressedThisFrame;
        openPressed = keyboard.digit8Key.wasPressedThisFrame;
        resetPressed = keyboard.digit0Key.wasPressedThisFrame;
#else
        s1Negative = Input.GetKey(KeyCode.Alpha1);
        s1Positive = Input.GetKey(KeyCode.Alpha2);
        s2Negative = Input.GetKey(KeyCode.Alpha3);
        s2Positive = Input.GetKey(KeyCode.Alpha4);
        s3Negative = Input.GetKey(KeyCode.Alpha5);
        s3Positive = Input.GetKey(KeyCode.Alpha6);
        s1NegativePressed = Input.GetKeyDown(KeyCode.Alpha1);
        s1PositivePressed = Input.GetKeyDown(KeyCode.Alpha2);
        s2NegativePressed = Input.GetKeyDown(KeyCode.Alpha3);
        s2PositivePressed = Input.GetKeyDown(KeyCode.Alpha4);
        s3NegativePressed = Input.GetKeyDown(KeyCode.Alpha5);
        s3PositivePressed = Input.GetKeyDown(KeyCode.Alpha6);
        closePressed = Input.GetKeyDown(KeyCode.Alpha7);
        openPressed = Input.GetKeyDown(KeyCode.Alpha8);
        resetPressed = Input.GetKeyDown(KeyCode.Alpha0);
#endif

        float armStep = armDegreesPerSecond * Time.unscaledDeltaTime;

        s1Angle += Axis(s1Negative, s1Positive) * armStep;

        s2Angle += Axis(s2Negative, s2Positive) * armStep;

        s3WristRollAngle += Axis(s3Negative, s3Positive) * armStep;

        // A quick tap must produce an obvious servo command too. Holding the
        // same keys continues to move smoothly after this five-degree step.
        const float tapStep = 5f;

        if (s1NegativePressed)
            s1Angle -= tapStep;
        if (s1PositivePressed)
            s1Angle += tapStep;

        if (s2NegativePressed)
            s2Angle -= tapStep;
        if (s2PositivePressed)
            s2Angle += tapStep;

        if (s3NegativePressed)
            s3WristRollAngle -= tapStep;
        if (s3PositivePressed)
            s3WristRollAngle += tapStep;

        // A real positional servo is given a target angle. One key press now
        // sends an unambiguous fully-open or fully-closed S4 target instead of
        // requiring the Game view to receive a continuously held key.
        if (closePressed)
            s4Closure = s4MaximumClosureLimit;

        if (openPressed)
            s4Closure = s4MinimumClosureCommand;

        if (resetPressed)
            ResetToNeutral();

        ClampCommands();
    }

    private static float Axis(bool negative, bool positive)
    {
        return (positive ? 1f : 0f) - (negative ? 1f : 0f);
    }

    private void ClampCommands()
    {
        NormalizeAllowedAngleRange(
            ref s1MinimumAngle,
            ref s1MaximumAngle);
        NormalizeAllowedAngleRange(
            ref s2MinimumAngle,
            ref s2MaximumAngle);
        NormalizeAllowedAngleRange(
            ref s3MinimumAngle,
            ref s3MaximumAngle);
        NormalizePositiveRange(
            ref s4MinimumClosureCommand,
            ref s4MaximumClosureLimit);

        s1Angle = Mathf.Clamp(
            s1Angle,
            s1MinimumAngle,
            s1MaximumAngle);
        s2Angle = Mathf.Clamp(
            s2Angle,
            s2MinimumAngle,
            s2MaximumAngle);
        s3WristRollAngle = Mathf.Clamp(
            s3WristRollAngle,
            s3MinimumAngle,
            s3MaximumAngle);
        s4Closure = Mathf.Clamp(
            s4Closure,
            s4MinimumClosureCommand,
            s4MaximumClosureLimit);
        if (IsS4Binary)
        {
            s4Closure = S4NormalizedClosure >= 0.5f
                ? s4MaximumClosureLimit
                : s4MinimumClosureCommand;
        }

        closedJawOpeningAngle = Mathf.Clamp(
            closedJawOpeningAngle,
            0f,
            15f);
        openJawOpeningAngle = Mathf.Clamp(
            openJawOpeningAngle,
            Mathf.Max(20f, closedJawOpeningAngle + 1f),
            45f);
    }

    private static void NormalizeAllowedAngleRange(
        ref float minimum,
        ref float maximum)
    {
        // Commands are offsets from the imported neutral pose, so zero must
        // always remain reachable by the Reset button.
        minimum = Mathf.Clamp(minimum, -180f, 0f);
        maximum = Mathf.Clamp(maximum, 0f, 180f);

        if (maximum - minimum < 1f)
            maximum = Mathf.Min(180f, minimum + 1f);
    }

    private static void NormalizePositiveRange(
        ref float minimum,
        ref float maximum)
    {
        minimum = Mathf.Clamp(minimum, 0f, 179f);
        maximum = Mathf.Clamp(maximum, 1f, 180f);

        if (maximum - minimum < 1f)
            maximum = Mathf.Min(180f, minimum + 1f);
    }

    public void ConfigureRig(
        Transform s1,
        Transform rigidLink,
        Transform s2,
        Transform s3WristRoll,
        Transform upperPivot,
        Transform lowerPivot,
        Transform upper,
        Transform lower,
        Transform hold,
        Transform gripperIr,
        Vector3 wristRollLocalAxis,
        Transform[] rigidVisuals)
    {
        s1ShoulderPivot = s1;
        rigidArmLink = rigidLink;
        s2ElbowPivot = s2;
        s3WristRollPivot = s3WristRoll;
        upperJawPivot = upperPivot;
        lowerJawPivot = lowerPivot;
        upperJaw = upper;
        lowerJaw = lower;
        holdPoint = hold;
        gripperIrPoint = gripperIr;
        s3WristRollLocalAxis = wristRollLocalAxis.sqrMagnitude > 0.000001f
            ? wristRollLocalAxis.normalized
            : Vector3.right;

        s1Angle = 0f;
        s2Angle = 0f;
        s3WristRollAngle = 0f;
        s4Closure = s4MinimumClosureCommand;

        CaptureBindPose(rigidVisuals);
    }

    public void CaptureBindPose(Transform[] rigidVisuals)
    {
        if (s1ShoulderPivot == null ||
            rigidArmLink == null ||
            s2ElbowPivot == null ||
            s3WristRollPivot == null ||
            upperJawPivot == null ||
            lowerJawPivot == null ||
            upperJaw == null ||
            lowerJaw == null)
        {
            bindPoseCaptured = false;
            return;
        }

        CaptureTransform(
            s1ShoulderPivot,
            out s1LocalPosition,
            out s1NeutralRotation,
            out s1LocalScale);

        CaptureTransform(
            rigidArmLink,
            out rigidLinkLocalPosition,
            out rigidLinkNeutralRotation,
            out rigidLinkLocalScale);

        CaptureTransform(
            s2ElbowPivot,
            out s2LocalPosition,
            out s2NeutralRotation,
            out s2LocalScale);

        CaptureTransform(
            s3WristRollPivot,
            out s3WristRollLocalPosition,
            out s3WristRollNeutralRotation,
            out s3WristRollLocalScale);

        CaptureTransform(
            upperJawPivot,
            out upperJawPivotLocalPosition,
            out upperJawPivotNeutralRotation,
            out upperJawPivotLocalScale);

        CaptureTransform(
            lowerJawPivot,
            out lowerJawPivotLocalPosition,
            out lowerJawPivotNeutralRotation,
            out lowerJawPivotLocalScale);

        CaptureTransform(
            upperJaw,
            out upperJawLocalPosition,
            out upperJawNeutralRotation,
            out upperJawLocalScale);

        CaptureTransform(
            lowerJaw,
            out lowerJawLocalPosition,
            out lowerJawNeutralRotation,
            out lowerJawLocalScale);

        lockedVisualPoses = new LockedPose[rigidVisuals?.Length ?? 0];
        for (int index = 0; index < lockedVisualPoses.Length; index++)
        {
            Transform target = rigidVisuals[index];
            lockedVisualPoses[index] = new LockedPose
            {
                target = target,
                localPosition = target != null ? target.localPosition : Vector3.zero,
                localRotation = target != null ? target.localRotation : Quaternion.identity,
                localScale = target != null ? target.localScale : Vector3.one
            };
        }

        bindPoseCaptured = true;
        ApplyPose();
    }

    private static void CaptureTransform(
        Transform target,
        out Vector3 localPosition,
        out Quaternion localRotation,
        out Vector3 localScale)
    {
        localPosition = target.localPosition;
        localRotation = target.localRotation;
        localScale = target.localScale;
    }

    public void SetServoCommands(
        float requestedS1,
        float requestedFormerS3,
        float requestedS4Closure)
    {
        // Compatibility overload: before the wrist repair, the upstream S2
        // bend was mistakenly exposed as S3. Existing callers keep moving
        // that same joint, while the real wrist roll remains unchanged.
        s1Angle = requestedS1;
        s2Angle = requestedFormerS3;
        s4Closure = requestedS4Closure;
        ClampCommands();
        ApplyPose();
    }

    public void SetServoCommands(
        float requestedS1,
        float requestedS2,
        float requestedS3,
        float requestedS4Closure)
    {
        s1Angle = requestedS1;
        s2Angle = requestedS2;
        s3WristRollAngle = requestedS3;
        s4Closure = requestedS4Closure;
        ClampCommands();
        ApplyPose();
    }

    public void SetWristRoll(float requestedS3)
    {
        s3WristRollAngle = requestedS3;
        ClampCommands();
        ApplyPose();
    }

    public void SetS4ControlMode(S4JawControlMode mode)
    {
        s4ControlMode = mode;
        ClampCommands();
        ApplyPose();
    }

    public void ConfigureS4Positions(
        float openPositionDegrees,
        float closedPositionDegrees)
    {
        float closure01 = S4NormalizedClosure;
        s4MinimumClosureCommand = openPositionDegrees;
        s4MaximumClosureLimit = closedPositionDegrees;
        NormalizePositiveRange(
            ref s4MinimumClosureCommand,
            ref s4MaximumClosureLimit);
        s4Closure = Mathf.Lerp(
            s4MinimumClosureCommand,
            s4MaximumClosureLimit,
            closure01);
        ClampCommands();
        ApplyPose();
    }

    public void SetJawOpen()
    {
        s4Closure = s4MinimumClosureCommand;
        ClampCommands();
        ApplyPose();
    }

    public void SetJawClosed()
    {
        s4Closure = s4MaximumClosureLimit;
        ClampCommands();
        ApplyPose();
    }

    public void SetJawClosed(bool closed)
    {
        if (closed)
            SetJawClosed();
        else
            SetJawOpen();
    }

    /// <summary>
    /// Straight arm (S2=0). Only S1 shoulder sets height. S3 locked to floor pose.
    /// </summary>
    public void SetFloorPickupPose()
    {
        s1Angle = floorPickupS1;
        s2Angle = floorPickupS2;
        s3WristRollAngle = floorPickupS3;
        s4Closure = s4MinimumClosureCommand;
        ClampCommands();
        ApplyPose();
        EnforceFloorClearance();
    }

    /// <summary>
    /// Sets S1/S2 while locking wrist roll to the floor-pickup S3 value.
    /// </summary>
    public void SetArmPoseLockedWrist(float requestedS1, float requestedS2, float requestedS4)
    {
        s1Angle = requestedS1;
        s2Angle = requestedS2;
        s3WristRollAngle = floorPickupS3;
        s4Closure = requestedS4;
        ClampCommands();
        ApplyPose();
        EnforceFloorClearance();
    }

    /// <summary>
    /// Raises only S1 until the claw tip is strictly above the floor by
    /// <see cref="minTipClearanceMetres"/>. Never raises past
    /// <see cref="MaxS1ForFloorClearance"/> so a bad floor raycast cannot
    /// fold the arm into the sky.
    /// </summary>
    public bool EnforceFloorClearance()
    {
        bool clamped = false;
        float s1Ceiling = MaxS1ForFloorClearance;

        for (int attempt = 0; attempt < 32; attempt++)
        {
            Physics.SyncTransforms();
            if (GetClawClearanceAboveFloor() >= minTipClearanceMetres)
                return clamped;

            if (s1Angle >= s1Ceiling - 0.01f)
                return clamped;

            s1Angle = Mathf.Min(s1Angle + 1.0f, s1Ceiling);
            ClampCommands();
            ApplyPose();
            clamped = true;
        }

        Physics.SyncTransforms();
        return clamped;
    }

    public float GetClawClearanceAboveFloor()
    {
        return GetClawLowestWorldY() - GetFloorSurfaceY();
    }

    private float GetFloorSurfaceY()
    {
        Vector3 origin = holdPoint != null
            ? holdPoint.position
            : (s1ShoulderPivot != null ? s1ShoulderPivot.position : transform.position);
        origin.y += 0.35f;

        int hitCount = Physics.RaycastNonAlloc(
            origin,
            Vector3.down,
            floorHitBuffer,
            2.5f,
            floorRaycastLayers,
            QueryTriggerInteraction.Ignore);

        float bestY = float.PositiveInfinity;
        int count = Mathf.Min(hitCount, floorHitBuffer.Length);
        for (int index = 0; index < count; index++)
        {
            RaycastHit hit = floorHitBuffer[index];
            if (hit.collider == null)
                continue;

            // Ray often starts above the claw and would otherwise "see" the
            // jaw as the floor, then keep raising S1 until the arm points up.
            if (hit.collider.transform.IsChildOf(transform))
                continue;

            bestY = Mathf.Min(bestY, hit.point.y);
        }

        return float.IsPositiveInfinity(bestY) ? 0f : bestY;
    }

    private float GetClawLowestWorldY()
    {
        float lowest = float.PositiveInfinity;
        AccumulateSubtreeLowestWorldY(s3WristRollPivot, ref lowest);
        AccumulateSubtreeLowestWorldY(upperJawPivot, ref lowest);
        AccumulateSubtreeLowestWorldY(lowerJawPivot, ref lowest);
        AccumulateLowestWorldY(holdPoint, ref lowest);
        AccumulateLowestWorldY(gripperIrPoint, ref lowest);
        return float.IsPositiveInfinity(lowest) ? transform.position.y : lowest;
    }

    private static void AccumulateSubtreeLowestWorldY(Transform root, ref float lowest)
    {
        if (root == null)
            return;

        Renderer[] renderers = root.GetComponentsInChildren<Renderer>(true);
        for (int index = 0; index < renderers.Length; index++)
        {
            if (renderers[index] != null)
                lowest = Mathf.Min(lowest, renderers[index].bounds.min.y);
        }

        Collider[] colliders = root.GetComponentsInChildren<Collider>(true);
        for (int index = 0; index < colliders.Length; index++)
        {
            Collider collider = colliders[index];
            if (collider != null && collider.enabled)
                lowest = Mathf.Min(lowest, collider.bounds.min.y);
        }

        if (renderers.Length == 0 && colliders.Length == 0)
            lowest = Mathf.Min(lowest, root.position.y);
    }

    private static void AccumulateLowestWorldY(Transform part, ref float lowest)
    {
        if (part == null)
            return;

        Renderer renderer = part.GetComponent<Renderer>();
        if (renderer != null)
            lowest = Mathf.Min(lowest, renderer.bounds.min.y);

        Collider collider = part.GetComponent<Collider>();
        if (collider != null && collider.enabled)
            lowest = Mathf.Min(lowest, collider.bounds.min.y);

        if (renderer == null && collider == null)
            lowest = Mathf.Min(lowest, part.position.y);
    }

    [ContextMenu("Reset Servo Axes To Neutral")]
    public void ResetToNeutral()
    {
        s1Angle = 0f;
        s2Angle = 0f;
        s3WristRollAngle = 0f;
        s4Closure = s4MinimumClosureCommand;
        ClampCommands();
        ApplyPose();
    }

    public void SetJawSigns(float upperSign, float lowerSign)
    {
        upperJawSign = Mathf.Sign(upperSign);
        lowerJawSign = Mathf.Sign(lowerSign);
        ApplyPose();
    }

    public void ApplyPose()
    {
        // Inspector edits to the allowed ranges take effect immediately,
        // including when the scene is not in Play mode.
        ClampCommands();

        if (!bindPoseCaptured)
            return;

        ApplyJoint(
            s1ShoulderPivot,
            s1LocalPosition,
            s1NeutralRotation,
            s1LocalScale,
            s1LocalAxis,
            s1Angle);

        ApplyJoint(
            rigidArmLink,
            rigidLinkLocalPosition,
            rigidLinkNeutralRotation,
            rigidLinkLocalScale,
            Vector3.forward,
            0f);

        ApplyJoint(
            s2ElbowPivot,
            s2LocalPosition,
            s2NeutralRotation,
            s2LocalScale,
            s2LocalAxis,
            s2Angle);

        // S3 is the independent axial wrist servo. Only the claw-side
        // assembly is below this pivot, so it rolls without moving the servo
        // housing or pulling any gripper part away from its shaft.
        ApplyJoint(
            s3WristRollPivot,
            s3WristRollLocalPosition,
            s3WristRollNeutralRotation,
            s3WristRollLocalScale,
            s3WristRollLocalAxis,
            s3WristRollAngle);

        foreach (LockedPose pose in lockedVisualPoses)
        {
            if (pose.target == null)
                continue;

            pose.target.localPosition = pose.localPosition;
            pose.target.localRotation = pose.localRotation;
            pose.target.localScale = pose.localScale;
        }

        // The S4 servo endpoints are editable in Unity and remain distinct
        // from the per-finger mesh hinge angles. Binary mode stores exactly
        // one endpoint; continuous mode may interpolate between them.
        float closure = S4NormalizedClosure;
        float jawOpeningAngle = Mathf.Lerp(
            openJawOpeningAngle,
            closedJawOpeningAngle,
            closure);

        ApplyJoint(
            upperJawPivot,
            upperJawPivotLocalPosition,
            upperJawPivotNeutralRotation,
            upperJawPivotLocalScale,
            jawPivotLocalAxis,
            upperJawSign * jawOpeningAngle);

        ApplyJoint(
            lowerJawPivot,
            lowerJawPivotLocalPosition,
            lowerJawPivotNeutralRotation,
            lowerJawPivotLocalScale,
            jawPivotLocalAxis,
            lowerJawSign * jawOpeningAngle);

        // The visual meshes never rotate independently. Their transforms are
        // rigidly locked below the two physical hinge objects, so the circular
        // hinge bosses remain on their shafts throughout S4 travel.
        ApplyJoint(
            upperJaw,
            upperJawLocalPosition,
            upperJawNeutralRotation,
            upperJawLocalScale,
            Vector3.up,
            0f);

        ApplyJoint(
            lowerJaw,
            lowerJawLocalPosition,
            lowerJawNeutralRotation,
            lowerJawLocalScale,
            Vector3.up,
            0f);
    }

    private static void ApplyJoint(
        Transform joint,
        Vector3 localPosition,
        Quaternion neutralRotation,
        Vector3 localScale,
        Vector3 axis,
        float angle)
    {
        if (joint == null)
            return;

        joint.localPosition = localPosition;
        joint.localScale = localScale;
        joint.localRotation =
            neutralRotation * Quaternion.AngleAxis(angle, axis.normalized);
    }

    private void OnGUI()
    {
        if (!Application.isPlaying || !showControlsOverlay)
            return;

        const float width = 350f;
        const float height = 202f;

        GUI.Box(new Rect(12f, 12f, width, height), "GFS-X SERVOS");

        float requestedS1 = DrawServoRow(
            "S1 shoulder  1/2",
            s1Angle,
            s1MinimumAngle,
            s1MaximumAngle,
            43f);
        float requestedS2 = DrawServoRow(
            "S2 elbow     3/4",
            s2Angle,
            s2MinimumAngle,
            s2MaximumAngle,
            71f);
        float requestedS3 = DrawServoRow(
            "S3 wrist roll 5/6",
            s3WristRollAngle,
            s3MinimumAngle,
            s3MaximumAngle,
            99f);
        float requestedS4 = IsS4Binary
            ? DrawBinaryJawRow(
                "S4 jaws      8/7",
                s4Closure,
                s4MinimumClosureCommand,
                s4MaximumClosureLimit,
                127f)
            : DrawServoRow(
                "S4 jaws      8/7",
                s4Closure,
                s4MinimumClosureCommand,
                s4MaximumClosureLimit,
                127f);

        bool servoChanged =
            !Mathf.Approximately(requestedS1, s1Angle) ||
            !Mathf.Approximately(requestedS2, s2Angle) ||
            !Mathf.Approximately(requestedS3, s3WristRollAngle) ||
            !Mathf.Approximately(requestedS4, s4Closure);

        if (servoChanged)
        {
            SetServoCommands(
                requestedS1,
                requestedS2,
                requestedS3,
                requestedS4);
        }

        if (GUI.Button(new Rect(24f, 163f, 104f, 26f), "RESET"))
            ResetToNeutral();

        GUI.Label(
            new Rect(140f, 166f, 208f, 22f),
            IsS4Binary
                ? $"OPEN {s4MinimumClosureCommand:F1}° | " +
                  $"CLOSED {s4MaximumClosureLimit:F1}°"
                : "S4 continuous: OPEN  ← slider →  CLOSED");
    }

    private static float DrawBinaryJawRow(
        string label,
        float value,
        float openPosition,
        float closedPosition,
        float y)
    {
        GUI.Label(new Rect(24f, y, 112f, 22f), label);

        int currentState = Mathf.InverseLerp(
            openPosition,
            closedPosition,
            value) >= 0.5f
            ? 1
            : 0;
        int requestedState = GUI.Toolbar(
            new Rect(140f, y, 154f, 22f),
            currentState,
            BinaryJawLabels);
        float requestedValue = requestedState == 0
            ? openPosition
            : closedPosition;

        GUI.Label(
            new Rect(302f, y, 54f, 22f),
            $"{requestedValue:F1}°");
        return requestedValue;
    }

    private static float DrawServoRow(
        string label,
        float value,
        float minimum,
        float maximum,
        float y)
    {
        GUI.Label(new Rect(24f, y, 112f, 22f), label);

        value = GUI.HorizontalSlider(
            new Rect(140f, y + 6f, 154f, 18f),
            value,
            minimum,
            maximum);

        value = Mathf.Clamp(value, minimum, maximum);
        GUI.Label(new Rect(302f, y, 54f, 22f), $"{value:F1}°");
        return value;
    }
}
