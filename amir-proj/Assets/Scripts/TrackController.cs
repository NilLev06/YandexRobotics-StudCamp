using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Differential (skid-steer) drive for the GFS-X tracked base.
/// It models two motor commands but moves the stable chassis Rigidbody,
/// so WheelColliders are intentionally not required.
/// </summary>
[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public sealed class TrackController : MonoBehaviour
{
    public enum ForwardAxis
    {
        LocalX,
        LocalZ
    }

    [Header("References and orientation")]
    [SerializeField] private Rigidbody body;
    [SerializeField] private ForwardAxis forwardAxis = ForwardAxis.LocalX;
    [SerializeField] private bool invertForward = false;

    [Header("Keyboard")]
    [SerializeField] private bool readKeyboard = true;

    [Header("Differential drive calibration")]
    [Tooltip("Unclamped linear-speed scale used for the gas command.")]
    [SerializeField, Min(0.01f)] private float moveSpeed = 0.57f;

    [Tooltip("Maximum chassis yaw rate in degrees per second.")]
    [SerializeField, Min(1f)] private float turnSpeed = 120f;

    [Tooltip("Full-steer speed contribution applied oppositely to the tracks, in m/s.")]
    [SerializeField, Range(0f, 1f)] private float turnK = 0.30f;

    [Tooltip("Maximum requested chassis linear speed in metres per second.")]
    [SerializeField, Min(0.01f)] private float maxLinearCmd = 0.25f;

    [Header("Motor / PWM model")]
    [Tooltip("Conversion used by the reference robot: metres/second to PWM percent.")]
    [SerializeField, Min(1f)] private float metersPerSecondToPwm = 200f;

    [Tooltip("PWM magnitudes below this value do not move a motor.")]
    [SerializeField, Range(0f, 99f)] private float motorDeadzone = 10f;

    [Tooltip("Smallest PWM magnitude that can start a real motor.")]
    [SerializeField, Range(0f, 100f)] private float minMotorPwm = 35f;

    [Tooltip("Largest PWM change permitted per physics tick.")]
    [SerializeField, Range(0.1f, 100f)] private float maxPwmStep = 15f;

    [Header("Rigidbody stability")]
    [SerializeField, Min(0f)] private float linearDamping = 8f;
    [SerializeField, Min(0f)] private float angularDamping = 8f;

    private float gasCommand;
    private float steeringCommand;
    private float leftPwm;
    private float rightPwm;
    private float leftTrackSpeed;
    private float rightTrackSpeed;
    private float linearSpeed;
    private float yawRateDegrees;

    public float GasCommand => gasCommand;
    public float SteeringCommand => steeringCommand;
    public float LeftPwm => leftPwm;
    public float RightPwm => rightPwm;
    public float LeftTrackSpeed => leftTrackSpeed;
    public float RightTrackSpeed => rightTrackSpeed;
    public float LinearSpeed => linearSpeed;
    public float YawRateDegrees => yawRateDegrees;
    public bool KeyboardControlEnabled
    {
        get => readKeyboard;
        set
        {
            readKeyboard = value;

            if (readKeyboard)
                SetCommand(0f, 0f);
        }
    }

    public Vector3 WorldForward
    {
        get
        {
            Quaternion rotation = body != null ? body.rotation : transform.rotation;
            Vector3 localForward = forwardAxis == ForwardAxis.LocalX
                ? Vector3.right
                : Vector3.forward;

            if (invertForward)
                localForward = -localForward;

            Vector3 worldForward = rotation * localForward;
            worldForward.y = 0f;
            return worldForward.sqrMagnitude > 0.000001f
                ? worldForward.normalized
                : Vector3.right;
        }
    }

    private void Reset()
    {
        body = GetComponent<Rigidbody>();
    }

    private void Awake()
    {
        if (body == null)
            body = GetComponent<Rigidbody>();

        ConfigureRigidbody();
        ClearMotorState();
    }

    private void OnValidate()
    {
        if (body == null)
            body = GetComponent<Rigidbody>();

        moveSpeed = Mathf.Max(0.01f, moveSpeed);
        turnSpeed = Mathf.Max(1f, turnSpeed);
        turnK = Mathf.Clamp01(turnK);
        maxLinearCmd = Mathf.Max(0.01f, maxLinearCmd);
        metersPerSecondToPwm = Mathf.Max(1f, metersPerSecondToPwm);
        motorDeadzone = Mathf.Clamp(motorDeadzone, 0f, 99f);
        minMotorPwm = Mathf.Clamp(
            Mathf.Max(minMotorPwm, motorDeadzone),
            0f,
            100f);
        maxPwmStep = Mathf.Clamp(maxPwmStep, 0.1f, 100f);
        linearDamping = Mathf.Max(0f, linearDamping);
        angularDamping = Mathf.Max(0f, angularDamping);
    }

    private void Update()
    {
        if (!readKeyboard)
            return;

        bool forwardPressed;
        bool reversePressed;
        bool rightPressed;
        bool leftPressed;

#if ENABLE_INPUT_SYSTEM
        Keyboard keyboard = Keyboard.current;
        if (keyboard == null)
        {
            SetCommand(0f, 0f);
            return;
        }

        forwardPressed = keyboard.wKey.isPressed;
        reversePressed = keyboard.sKey.isPressed;
        rightPressed = keyboard.dKey.isPressed;
        leftPressed = keyboard.aKey.isPressed;
#else
        forwardPressed = Input.GetKey(KeyCode.W);
        reversePressed = Input.GetKey(KeyCode.S);
        rightPressed = Input.GetKey(KeyCode.D);
        leftPressed = Input.GetKey(KeyCode.A);
#endif

        float gas = ButtonValue(forwardPressed) -
                    ButtonValue(reversePressed);
        float steering = ButtonValue(rightPressed) -
                         ButtonValue(leftPressed);

        SetCommand(gas, steering);
    }

    private void FixedUpdate()
    {
        if (body == null)
            return;

        CalculateRequestedTrackSpeeds(
            out float requestedLeftSpeed,
            out float requestedRightSpeed);

        float requestedLeftPwm = SpeedToMotorPwm(requestedLeftSpeed);
        float requestedRightPwm = SpeedToMotorPwm(requestedRightSpeed);

        leftPwm = Mathf.MoveTowards(
            leftPwm,
            requestedLeftPwm,
            maxPwmStep);
        rightPwm = Mathf.MoveTowards(
            rightPwm,
            requestedRightPwm,
            maxPwmStep);

        leftTrackSpeed = leftPwm / metersPerSecondToPwm;
        rightTrackSpeed = rightPwm / metersPerSecondToPwm;
        linearSpeed = Mathf.Clamp(
            (leftTrackSpeed + rightTrackSpeed) * 0.5f,
            -maxLinearCmd,
            maxLinearCmd);

        float fullTurnTrackDifference = 2f * turnK;
        yawRateDegrees = fullTurnTrackDifference > 0.000001f
            ? Mathf.Clamp(
                  (leftTrackSpeed - rightTrackSpeed) /
                  fullTurnTrackDifference,
                  -1f,
                  1f) * turnSpeed
            : 0f;

        if (Mathf.Abs(linearSpeed) < 0.000001f &&
            Mathf.Abs(yawRateDegrees) < 0.000001f)
        {
            return;
        }

        body.WakeUp();

        float fixedDelta = Time.fixedDeltaTime;
        Vector3 nextPosition =
            body.position + WorldForward * (linearSpeed * fixedDelta);
        Quaternion yawStep = Quaternion.AngleAxis(
            yawRateDegrees * fixedDelta,
            Vector3.up);
        Quaternion nextRotation = yawStep * body.rotation;

        body.MovePosition(nextPosition);
        body.MoveRotation(nextRotation);
    }

    /// <summary>
    /// Sets normalized gas and steering commands in [-1, 1]. Disable
    /// Read Keyboard in the Inspector when ROS or ML-Agents calls this API.
    /// </summary>
    public void SetCommand(float gas, float steering)
    {
        gasCommand = SanitizeAndClamp(gas);
        steeringCommand = SanitizeAndClamp(steering);
    }

    /// <summary>
    /// Clears commands. Pass true for an immediate emergency stop; otherwise
    /// the configured PWM slew limit is respected.
    /// </summary>
    public void Stop(bool immediate = false)
    {
        SetCommand(0f, 0f);

        if (immediate)
        {
            ClearMotorState();

            if (body != null)
            {
                body.linearVelocity = Vector3.zero;
                body.angularVelocity = Vector3.zero;
            }
        }
    }

    private void OnDisable()
    {
        Stop(true);
    }

    private void CalculateRequestedTrackSpeeds(
        out float requestedLeftSpeed,
        out float requestedRightSpeed)
    {
        float requestedLinearSpeed = Mathf.Clamp(
            gasCommand * moveSpeed,
            -maxLinearCmd,
            maxLinearCmd);
        float requestedTurnSpeed =
            steeringCommand * turnK;

        // Facing the robot's local +X direction, local +Z is its left side.
        // A positive steering command therefore speeds up the left track and
        // slows/reverses the right track, producing a right-hand turn.
        requestedLeftSpeed = requestedLinearSpeed + requestedTurnSpeed;
        requestedRightSpeed = requestedLinearSpeed - requestedTurnSpeed;
    }

    private float SpeedToMotorPwm(float requestedSpeed)
    {
        float pwm = Mathf.Clamp(
            requestedSpeed * metersPerSecondToPwm,
            -100f,
            100f);
        float magnitude = Mathf.Abs(pwm);

        if (magnitude <= motorDeadzone)
            return 0f;

        magnitude = Mathf.Max(magnitude, minMotorPwm);
        return Mathf.Sign(pwm) * magnitude;
    }

    private void ConfigureRigidbody()
    {
        if (body == null)
            return;

        body.interpolation = RigidbodyInterpolation.Interpolate;
        body.collisionDetectionMode = CollisionDetectionMode.Continuous;
        body.linearDamping = linearDamping;
        body.angularDamping = angularDamping;
        body.constraints |=
            RigidbodyConstraints.FreezeRotationX |
            RigidbodyConstraints.FreezeRotationZ;
    }

    private void ClearMotorState()
    {
        gasCommand = 0f;
        steeringCommand = 0f;
        leftPwm = 0f;
        rightPwm = 0f;
        leftTrackSpeed = 0f;
        rightTrackSpeed = 0f;
        linearSpeed = 0f;
        yawRateDegrees = 0f;
    }

    private static float ButtonValue(bool pressed)
    {
        return pressed ? 1f : 0f;
    }

    private static float SanitizeAndClamp(float value)
    {
        return float.IsNaN(value) || float.IsInfinity(value)
            ? 0f
            : Mathf.Clamp(value, -1f, 1f);
    }
}
