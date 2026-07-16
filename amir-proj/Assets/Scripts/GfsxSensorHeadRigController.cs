using System;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Transform-driven S5/S6 pan/tilt rig for the GFS-X sensor head.
/// The imported neutral pose is captured once, then every moving part is
/// restored around the two modeled S5/S6 pivot axes so meshes cannot drift
/// apart.
/// </summary>
[ExecuteAlways]
[DefaultExecutionOrder(-200)]
[DisallowMultipleComponent]
public sealed class GfsxSensorHeadRigController : MonoBehaviour
{
    [Serializable]
    private struct LockedPose
    {
        public Transform target;
        public Vector3 localPosition;
        public Quaternion localRotation;
        public Vector3 localScale;
    }

    [Header("S5/S6 servo and sensor references")]
    [SerializeField] private Transform s5SensorYawPivot;
    [SerializeField] private Transform s6CameraPitchPivot;
    [SerializeField] private Transform ultrasoundVisual;
    [SerializeField] private Transform cameraBaseVisual;
    [SerializeField] private Transform cameraVisual;
    [SerializeField] private Transform ultrasonicBeamPoint;

    [Header("Commands (degrees from Unity reference; not hardware neutral)")]
    [SerializeField] private float s5PanAngle;
    [SerializeField] private float s6TiltAngle;

    [Header("Calibration limits (editable; not specified by the manual)")]
    [SerializeField, Range(-180f, 0f)]
    private float s5MinimumAngle = -75f;
    [SerializeField, Range(0f, 180f)]
    private float s5MaximumAngle = 75f;
    [SerializeField, Range(-180f, 0f)]
    private float s6MinimumAngle = -30f;
    [SerializeField, Range(0f, 180f)]
    private float s6MaximumAngle = 45f;

    [Header("Keyboard and compact HUD")]
    [SerializeField] private bool readKeyboard = true;
    [SerializeField, Min(1f)] private float degreesPerSecond = 45f;
    [SerializeField] private bool showControlsOverlay = true;

    [Header("Unity local servo axes (scene calibration)")]
    [SerializeField] private Vector3 s5YawLocalAxis = Vector3.up;
    [SerializeField] private Vector3 s6PitchLocalAxis = Vector3.forward;

    [Header("Captured mechanical bind pose")]
    [SerializeField, HideInInspector] private bool bindPoseCaptured;
    [SerializeField, HideInInspector] private Vector3 s5LocalPosition;
    [SerializeField, HideInInspector] private Quaternion s5NeutralRotation;
    [SerializeField, HideInInspector] private Vector3 s5LocalScale;
    [SerializeField, HideInInspector] private Vector3 s6LocalPosition;
    [SerializeField, HideInInspector] private Quaternion s6NeutralRotation;
    [SerializeField, HideInInspector] private Vector3 s6LocalScale;
    [SerializeField, HideInInspector]
    private LockedPose[] lockedVisualPoses = Array.Empty<LockedPose>();

    public bool IsConfigured =>
        bindPoseCaptured &&
        s5SensorYawPivot != null &&
        s6CameraPitchPivot != null &&
        ultrasoundVisual != null &&
        cameraBaseVisual != null &&
        cameraVisual != null &&
        ultrasonicBeamPoint != null;

    public float S5PanAngle => s5PanAngle;
    public float S6TiltAngle => s6TiltAngle;
    public float S5MinimumAngle => s5MinimumAngle;
    public float S5MaximumAngle => s5MaximumAngle;
    public float S6MinimumAngle => s6MinimumAngle;
    public float S6MaximumAngle => s6MaximumAngle;
    public Transform S5SensorYawPivot => s5SensorYawPivot;
    public Transform S6CameraPitchPivot => s6CameraPitchPivot;
    public Transform UltrasoundVisual => ultrasoundVisual;
    public Transform CameraBaseVisual => cameraBaseVisual;
    public Transform CameraVisual => cameraVisual;
    public Transform UltrasonicBeamPoint => ultrasonicBeamPoint;

    public bool KeyboardControlEnabled
    {
        get => readKeyboard;
        set => readKeyboard = value;
    }

    private void Update()
    {
        if (Application.isPlaying && readKeyboard)
            ReadKeyboard();

        ApplyPose();
    }

    private void LateUpdate()
    {
        // Keep the complete visual chain on its captured shafts after every
        // other runtime component has had a chance to update.
        ApplyPose();
    }

    private void OnValidate()
    {
        ClampCommandsAndRanges();

        // A disabled controller is the explicit pivot-editing mode. Do not
        // restore the old bind pose while the user moves an empty pivot and
        // reparents its visuals in the Editor.
        if (enabled)
            ApplyPose();
    }

    private void ReadKeyboard()
    {
        bool panNegative;
        bool panPositive;
        bool tiltNegative;
        bool tiltPositive;
        bool panNegativePressed;
        bool panPositivePressed;
        bool tiltNegativePressed;
        bool tiltPositivePressed;
        bool resetPressed;

#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;
        if (keyboard == null)
            return;

        panNegative =
            keyboard.jKey.isPressed || keyboard.leftArrowKey.isPressed;
        panPositive =
            keyboard.lKey.isPressed || keyboard.rightArrowKey.isPressed;
        tiltNegative =
            keyboard.kKey.isPressed || keyboard.downArrowKey.isPressed;
        tiltPositive =
            keyboard.iKey.isPressed || keyboard.upArrowKey.isPressed;
        panNegativePressed =
            keyboard.jKey.wasPressedThisFrame ||
            keyboard.leftArrowKey.wasPressedThisFrame;
        panPositivePressed =
            keyboard.lKey.wasPressedThisFrame ||
            keyboard.rightArrowKey.wasPressedThisFrame;
        tiltNegativePressed =
            keyboard.kKey.wasPressedThisFrame ||
            keyboard.downArrowKey.wasPressedThisFrame;
        tiltPositivePressed =
            keyboard.iKey.wasPressedThisFrame ||
            keyboard.upArrowKey.wasPressedThisFrame;
        resetPressed = keyboard.uKey.wasPressedThisFrame;
#else
        panNegative =
            Input.GetKey(KeyCode.J) || Input.GetKey(KeyCode.LeftArrow);
        panPositive =
            Input.GetKey(KeyCode.L) || Input.GetKey(KeyCode.RightArrow);
        tiltNegative =
            Input.GetKey(KeyCode.K) || Input.GetKey(KeyCode.DownArrow);
        tiltPositive =
            Input.GetKey(KeyCode.I) || Input.GetKey(KeyCode.UpArrow);
        panNegativePressed =
            Input.GetKeyDown(KeyCode.J) ||
            Input.GetKeyDown(KeyCode.LeftArrow);
        panPositivePressed =
            Input.GetKeyDown(KeyCode.L) ||
            Input.GetKeyDown(KeyCode.RightArrow);
        tiltNegativePressed =
            Input.GetKeyDown(KeyCode.K) ||
            Input.GetKeyDown(KeyCode.DownArrow);
        tiltPositivePressed =
            Input.GetKeyDown(KeyCode.I) ||
            Input.GetKeyDown(KeyCode.UpArrow);
        resetPressed = Input.GetKeyDown(KeyCode.U);
#endif

        float step = degreesPerSecond * Time.unscaledDeltaTime;
        s5PanAngle += Axis(panNegative, panPositive) * step;
        s6TiltAngle += Axis(tiltNegative, tiltPositive) * step;

        // A tap should remain visible even on a short rendered frame.
        const float tapStep = 5f;
        if (panNegativePressed)
            s5PanAngle -= tapStep;
        if (panPositivePressed)
            s5PanAngle += tapStep;
        if (tiltNegativePressed)
            s6TiltAngle -= tapStep;
        if (tiltPositivePressed)
            s6TiltAngle += tapStep;
        if (resetPressed)
            ResetToNeutral();

        ClampCommandsAndRanges();
    }

    private static float Axis(bool negative, bool positive)
    {
        return (positive ? 1f : 0f) - (negative ? 1f : 0f);
    }

    public void ConfigureRig(
        Transform s5Yaw,
        Transform s6Pitch,
        Transform ultrasound,
        Transform cameraBase,
        Transform camera,
        Transform beamPoint,
        Vector3 yawLocalAxis,
        Vector3 pitchLocalAxis,
        Transform[] rigidVisuals)
    {
        // A public reconfiguration call must not bake a commanded pose into
        // the new Unity reference pose. Restore the old references first.
        if (bindPoseCaptured)
            ResetToNeutral();

        s5SensorYawPivot = s5Yaw;
        s6CameraPitchPivot = s6Pitch;
        ultrasoundVisual = ultrasound;
        cameraBaseVisual = cameraBase;
        cameraVisual = camera;
        ultrasonicBeamPoint = beamPoint;
        s5YawLocalAxis = SafeAxis(yawLocalAxis, Vector3.up);
        s6PitchLocalAxis = SafeAxis(pitchLocalAxis, Vector3.forward);
        s5PanAngle = 0f;
        s6TiltAngle = 0f;
        CaptureBindPose(rigidVisuals);
    }

    public void CaptureBindPose(Transform[] rigidVisuals)
    {
        if (s5SensorYawPivot == null ||
            s6CameraPitchPivot == null ||
            ultrasoundVisual == null ||
            cameraBaseVisual == null ||
            cameraVisual == null ||
            ultrasonicBeamPoint == null)
        {
            bindPoseCaptured = false;
            return;
        }

        CaptureTransform(
            s5SensorYawPivot,
            out s5LocalPosition,
            out s5NeutralRotation,
            out s5LocalScale);
        CaptureTransform(
            s6CameraPitchPivot,
            out s6LocalPosition,
            out s6NeutralRotation,
            out s6LocalScale);

        CaptureLockedVisualPoses(rigidVisuals);

        bindPoseCaptured = true;
        ApplyPose();
    }

    /// <summary>
    /// Captures the transforms currently visible in the scene as the new
    /// mechanical reference without baking the current S5/S6 command angles
    /// into neutral. This is the safe final step after relocating an empty
    /// pivot while this component is disabled.
    /// </summary>
    public bool CaptureCurrentReferencePose()
    {
        ClampCommandsAndRanges();
        if (s5SensorYawPivot == null ||
            s6CameraPitchPivot == null ||
            ultrasoundVisual == null ||
            cameraBaseVisual == null ||
            cameraVisual == null ||
            ultrasonicBeamPoint == null)
        {
            bindPoseCaptured = false;
            return false;
        }

        Vector3 yawAxis = SafeAxis(s5YawLocalAxis, Vector3.up);
        Vector3 pitchAxis = SafeAxis(
            s6PitchLocalAxis,
            Vector3.forward);

        s5LocalPosition = s5SensorYawPivot.localPosition;
        s5LocalScale = s5SensorYawPivot.localScale;
        s5NeutralRotation = s5SensorYawPivot.localRotation *
            Quaternion.Inverse(
                Quaternion.AngleAxis(s5PanAngle, yawAxis));

        s6LocalPosition = s6CameraPitchPivot.localPosition;
        s6LocalScale = s6CameraPitchPivot.localScale;
        s6NeutralRotation = s6CameraPitchPivot.localRotation *
            Quaternion.Inverse(
                Quaternion.AngleAxis(s6TiltAngle, pitchAxis));

        CaptureLockedVisualPoses(new[]
        {
            ultrasoundVisual,
            cameraBaseVisual,
            cameraVisual,
            ultrasonicBeamPoint
        });

        bindPoseCaptured = true;
        ApplyPose();
        return true;
    }

    public void SetServoCommands(float requestedS5Pan, float requestedS6Tilt)
    {
        s5PanAngle = requestedS5Pan;
        s6TiltAngle = requestedS6Tilt;
        ClampCommandsAndRanges();
        ApplyPose();
    }

    public void SetPan(float requestedS5Pan)
    {
        SetServoCommands(requestedS5Pan, s6TiltAngle);
    }

    public void SetTilt(float requestedS6Tilt)
    {
        SetServoCommands(s5PanAngle, requestedS6Tilt);
    }

    [ContextMenu("Reset S5/S6 To Neutral")]
    public void ResetToNeutral()
    {
        s5PanAngle = 0f;
        s6TiltAngle = 0f;
        ClampCommandsAndRanges();
        ApplyPose();
    }

    public void ApplyPose()
    {
        ClampCommandsAndRanges();
        if (!bindPoseCaptured)
            return;

        ApplyJoint(
            s5SensorYawPivot,
            s5LocalPosition,
            s5NeutralRotation,
            s5LocalScale,
            s5YawLocalAxis,
            s5PanAngle);
        ApplyJoint(
            s6CameraPitchPivot,
            s6LocalPosition,
            s6NeutralRotation,
            s6LocalScale,
            s6PitchLocalAxis,
            s6TiltAngle);

        foreach (LockedPose pose in lockedVisualPoses)
        {
            if (pose.target == null)
                continue;

            pose.target.localPosition = pose.localPosition;
            pose.target.localRotation = pose.localRotation;
            pose.target.localScale = pose.localScale;
        }
    }

    private void ClampCommandsAndRanges()
    {
        NormalizeRange(ref s5MinimumAngle, ref s5MaximumAngle);
        NormalizeRange(ref s6MinimumAngle, ref s6MaximumAngle);
        s5PanAngle = Mathf.Clamp(
            s5PanAngle,
            s5MinimumAngle,
            s5MaximumAngle);
        s6TiltAngle = Mathf.Clamp(
            s6TiltAngle,
            s6MinimumAngle,
            s6MaximumAngle);
        degreesPerSecond = Mathf.Max(1f, degreesPerSecond);
        s5YawLocalAxis = SafeAxis(s5YawLocalAxis, Vector3.up);
        s6PitchLocalAxis = SafeAxis(
            s6PitchLocalAxis,
            Vector3.forward);
    }

    private static void NormalizeRange(ref float minimum, ref float maximum)
    {
        // Zero is the captured neutral pose and must always stay reachable.
        minimum = Mathf.Clamp(minimum, -180f, 0f);
        maximum = Mathf.Clamp(maximum, 0f, 180f);
        if (maximum - minimum < 1f)
            maximum = Mathf.Min(180f, minimum + 1f);
    }

    private static Vector3 SafeAxis(Vector3 requested, Vector3 fallback)
    {
        return requested.sqrMagnitude > 0.000001f
            ? requested.normalized
            : fallback;
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

    private void CaptureLockedVisualPoses(Transform[] rigidVisuals)
    {
        lockedVisualPoses = new LockedPose[rigidVisuals?.Length ?? 0];
        for (int index = 0; index < lockedVisualPoses.Length; index++)
        {
            Transform target = rigidVisuals[index];
            lockedVisualPoses[index] = new LockedPose
            {
                target = target,
                localPosition = target != null
                    ? target.localPosition
                    : Vector3.zero,
                localRotation = target != null
                    ? target.localRotation
                    : Quaternion.identity,
                localScale = target != null
                    ? target.localScale
                    : Vector3.one
            };
        }
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

        const float width = 282f;
        const float height = 91f;
        float x = Mathf.Max(12f, Screen.width - width - 12f);
        const float y = 12f;

        GUI.Box(
            new Rect(x, y, width, height),
            "SENSOR SERVOS  •  U = CENTER");

        float requestedS5 = DrawServoRow(
            "S5 pan  ←/→",
            s5PanAngle,
            s5MinimumAngle,
            s5MaximumAngle,
            x,
            y + 31f);
        float requestedS6 = DrawServoRow(
            "S6 tilt  ↓/↑",
            s6TiltAngle,
            s6MinimumAngle,
            s6MaximumAngle,
            x,
            y + 58f);

        if (!Mathf.Approximately(requestedS5, s5PanAngle) ||
            !Mathf.Approximately(requestedS6, s6TiltAngle))
        {
            SetServoCommands(requestedS5, requestedS6);
        }
    }

    private static float DrawServoRow(
        string label,
        float value,
        float minimum,
        float maximum,
        float x,
        float y)
    {
        GUI.Label(new Rect(x + 12f, y, 88f, 22f), label);
        value = GUI.HorizontalSlider(
            new Rect(x + 104f, y + 6f, 118f, 18f),
            value,
            minimum,
            maximum);
        value = Mathf.Clamp(value, minimum, maximum);
        GUI.Label(
            new Rect(x + 224f, y, 54f, 22f),
            $"{value:F1}°");
        return value;
    }
}
