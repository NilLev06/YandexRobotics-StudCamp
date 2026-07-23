using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Read-only physical telemetry and command-based dead reckoning for the
/// real GFS-X. This component never registers a publisher.
/// </summary>
[DefaultExecutionOrder(-250)]
[DisallowMultipleComponent]
public sealed class GfsxPhysicalTelemetry : MonoBehaviour
{
    private const int ServoCount = 6;
    private const string SensorTopic = "/sensor/data";
    private const string RearIrTopic = "/sensor/rear_ir";
    private const string MotorPwmTopic = "/gfsx/motor_pwm";
    private const string ServoStateTopic = "/gfsx/servo_state_degrees";
    private const string ServoArmedTopic = "/gfsx/servo_armed";
    private const string HardwareStatusTopic = "/gfsx/hardware_status";

    [Header("ROS-TCP Endpoint on Raspberry Pi")]
    [SerializeField] private string rosIpAddress = "192.168.2.152";
    [SerializeField, Range(1, 65535)] private int rosPort = 10000;

    [Header("Freshness gates")]
    [SerializeField, Range(0.15f, 2f)] private float sensorTimeoutSeconds = 0.45f;
    [SerializeField, Range(0.15f, 2f)] private float motorPwmTimeoutSeconds = 0.55f;
    [SerializeField, Range(0.25f, 3f)] private float servoTimeoutSeconds = 1.25f;

    [Header("Policy normalization")]
    [SerializeField, Min(0.1f)] private float ultrasonicRangeMetres = 3.43f;
    [SerializeField] private float sensorPanMinimumDegrees = 15f;
    [SerializeField] private float sensorPanMaximumDegrees = 160f;

    [Header("Estimated odometry (no wheel encoders are installed)")]
    [SerializeField, Min(1f)] private float pwmPerMetrePerSecond = 200f;
    [SerializeField, Min(0.01f)] private float turnTrackContribution = 0.30f;
    [SerializeField, Min(1f)] private float simulatedMaximumYawRateDegrees = 120f;

    private readonly object stateLock = new object();
    private readonly float[] servoState = new float[ServoCount];
    private ROSConnection ros;
    private float ultrasonicMetres;
    private float leftIr;
    private float rightIr;
    private float gripperIr;
    private float rearIr;
    private float leftPwm;
    private float rightPwm;
    private bool servoArmed;
    private string hardwareStatus = "No status received.";
    private bool hasSensor;
    private bool hasRearIr;
    private bool hasMotorPwm;
    private bool hasServoState;
    private bool hasServoArmed;
    private float sensorTime = float.NegativeInfinity;
    private float rearIrTime = float.NegativeInfinity;
    private float motorPwmTime = float.NegativeInfinity;
    private float servoStateTime = float.NegativeInfinity;
    private float servoArmedTime = float.NegativeInfinity;
    private Vector2 estimatedDisplacement;
    private float estimatedHeadingDegrees;
    private float estimatedSpeedMetresPerSecond;
    private float estimatedSignedSpeedMetresPerSecond;
    private int sensorPacketCount;

    public string RosIpAddress => rosIpAddress;
    public int RosPort => rosPort;
    public bool RosConnected =>
        ros != null && ros.HasConnectionThread && !ros.HasConnectionError;
    public bool SensorFresh => Fresh(hasSensor, sensorTime, sensorTimeoutSeconds);
    public bool RearIrFresh => Fresh(hasRearIr, rearIrTime, sensorTimeoutSeconds);
    public bool MotorPwmFresh => Fresh(hasMotorPwm, motorPwmTime, motorPwmTimeoutSeconds);
    public bool ServoStateFresh => Fresh(hasServoState, servoStateTime, servoTimeoutSeconds);
    public bool ServoArmedAckFresh => Fresh(hasServoArmed, servoArmedTime, servoTimeoutSeconds);
    public int SensorPacketCount => sensorPacketCount;
    public float UltrasonicMetres => SensorFresh ? ultrasonicMetres : 0f;
    public float UltrasonicNormalized => SensorFresh
        ? Mathf.Clamp01(ultrasonicMetres / ultrasonicRangeMetres)
        : 0f;
    public float LeftIr => SensorFresh ? leftIr : 1f;
    public float RightIr => SensorFresh ? rightIr : 1f;
    public float GripperIr => SensorFresh ? gripperIr : 0f;
    public float RearIr => RearIrFresh ? rearIr : 1f;
    public bool ServoArmed => ServoArmedAckFresh && servoArmed;
    public string HardwareStatus => hardwareStatus;
    public Vector2 EstimatedDisplacement => estimatedDisplacement;
    public float EstimatedHeadingDegrees => estimatedHeadingDegrees;
    public float EstimatedSpeedMetresPerSecond => estimatedSpeedMetresPerSecond;
    public float EstimatedSignedSpeedMetresPerSecond =>
        estimatedSignedSpeedMetresPerSecond;
    public float LeftPwm => MotorPwmFresh ? leftPwm : 0f;
    public float RightPwm => MotorPwmFresh ? rightPwm : 0f;
    public float SensorAgeSeconds => Age(hasSensor, sensorTime);
    public float MotorPwmAgeSeconds => Age(hasMotorPwm, motorPwmTime);
    public float ServoAgeSeconds => Age(hasServoState, servoStateTime);

    private void Awake()
    {
        Application.runInBackground = true;
        ros = ROSConnection.GetOrCreateInstance();
        ros.RosIPAddress = rosIpAddress;
        ros.RosPort = rosPort;
        ros.ConnectOnStart = true;
        ros.ShowHud = false;
        ros.listenForTFMessages = false;
        ros.TFTopics = System.Array.Empty<string>();
        RegisterSubscribers();
        ResetDeadReckoning();
    }

    private void Update()
    {
        IntegrateEstimatedOdometry(Time.unscaledDeltaTime);
    }

    private void OnValidate()
    {
        rosPort = Mathf.Clamp(rosPort, 1, 65535);
        sensorTimeoutSeconds = Mathf.Clamp(sensorTimeoutSeconds, 0.15f, 2f);
        motorPwmTimeoutSeconds = Mathf.Clamp(motorPwmTimeoutSeconds, 0.15f, 2f);
        servoTimeoutSeconds = Mathf.Clamp(servoTimeoutSeconds, 0.25f, 3f);
        ultrasonicRangeMetres = Mathf.Max(0.1f, ultrasonicRangeMetres);
        if (sensorPanMaximumDegrees < sensorPanMinimumDegrees)
        {
            float swap = sensorPanMinimumDegrees;
            sensorPanMinimumDegrees = sensorPanMaximumDegrees;
            sensorPanMaximumDegrees = swap;
        }
        pwmPerMetrePerSecond = Mathf.Max(1f, pwmPerMetrePerSecond);
        turnTrackContribution = Mathf.Max(0.01f, turnTrackContribution);
        simulatedMaximumYawRateDegrees = Mathf.Max(1f, simulatedMaximumYawRateDegrees);
    }

    public void Configure(string ipAddress, int port)
    {
        rosIpAddress = ipAddress;
        rosPort = Mathf.Clamp(port, 1, 65535);
    }

    public void ConfigureFreshness(
        float sensorSeconds,
        float motorPwmSeconds,
        float servoSeconds)
    {
        sensorTimeoutSeconds = Mathf.Clamp(sensorSeconds, 0.15f, 2f);
        motorPwmTimeoutSeconds = Mathf.Clamp(motorPwmSeconds, 0.15f, 2f);
        servoTimeoutSeconds = Mathf.Clamp(servoSeconds, 0.25f, 3f);
    }

    public void ResetDeadReckoning()
    {
        estimatedDisplacement = Vector2.zero;
        estimatedHeadingDegrees = 0f;
        estimatedSpeedMetresPerSecond = 0f;
        estimatedSignedSpeedMetresPerSecond = 0f;
    }

    public bool TryCopyServoState(float[] destination)
    {
        if (destination == null || destination.Length != ServoCount ||
            !ServoStateFresh)
        {
            return false;
        }
        lock (stateLock)
            System.Array.Copy(servoState, destination, ServoCount);
        return true;
    }

    public bool TryGetSensorPanDegrees(out float degrees)
    {
        degrees = 0f;
        if (!ServoStateFresh)
            return false;
        lock (stateLock)
            degrees = servoState[4];
        return true;
    }

    public float SensorPanNormalized
    {
        get
        {
            if (!ServoStateFresh)
                return 0f;
            float pan;
            lock (stateLock)
                pan = servoState[4];
            float range = sensorPanMaximumDegrees - sensorPanMinimumDegrees;
            if (range <= 0.0001f)
                return 0f;
            return Mathf.Clamp(
                Mathf.InverseLerp(
                    sensorPanMinimumDegrees,
                    sensorPanMaximumDegrees,
                    pan) * 2f - 1f,
                -1f,
                1f);
        }
    }

    private void RegisterSubscribers()
    {
        if (!ros.HasSubscriber(SensorTopic))
            ros.Subscribe<QuaternionMsg>(SensorTopic, ReceiveSensor);
        if (!ros.HasSubscriber(RearIrTopic))
            ros.Subscribe<Int32Msg>(RearIrTopic, ReceiveRearIr);
        if (!ros.HasSubscriber(MotorPwmTopic))
            ros.Subscribe<Vector3Msg>(MotorPwmTopic, ReceiveMotorPwm);
        if (!ros.HasSubscriber(ServoStateTopic))
            ros.Subscribe<Float32MultiArrayMsg>(ServoStateTopic, ReceiveServoState);
        if (!ros.HasSubscriber(ServoArmedTopic))
            ros.Subscribe<BoolMsg>(ServoArmedTopic, ReceiveServoArmed);
        if (!ros.HasSubscriber(HardwareStatusTopic))
            ros.Subscribe<StringMsg>(HardwareStatusTopic, ReceiveHardwareStatus);
    }

    private void ReceiveSensor(QuaternionMsg message)
    {
        if (message == null || !Finite(message.x) || !Finite(message.y) ||
            !Finite(message.z) || !Finite(message.w))
            return;
        ultrasonicMetres = Mathf.Clamp((float)message.x, 0f, ultrasonicRangeMetres);
        leftIr = message.y > 0.5 ? 1f : 0f;
        rightIr = message.z > 0.5 ? 1f : 0f;
        gripperIr = message.w > 0.5 ? 1f : 0f;
        hasSensor = true;
        sensorTime = Time.realtimeSinceStartup;
        sensorPacketCount++;
    }

    private void ReceiveRearIr(Int32Msg message)
    {
        if (message == null)
            return;
        rearIr = message.data != 0 ? 1f : 0f;
        hasRearIr = true;
        rearIrTime = Time.realtimeSinceStartup;
    }

    private void ReceiveMotorPwm(Vector3Msg message)
    {
        if (message == null || !Finite(message.x) || !Finite(message.y))
            return;
        leftPwm = Mathf.Clamp((float)message.x, -100f, 100f);
        rightPwm = Mathf.Clamp((float)message.y, -100f, 100f);
        hasMotorPwm = true;
        motorPwmTime = Time.realtimeSinceStartup;
    }

    private void ReceiveServoState(Float32MultiArrayMsg message)
    {
        if (message?.data == null || message.data.Length != ServoCount)
            return;
        for (int index = 0; index < ServoCount; index++)
        {
            if (!Finite(message.data[index]))
                return;
        }
        lock (stateLock)
            System.Array.Copy(message.data, servoState, ServoCount);
        hasServoState = true;
        servoStateTime = Time.realtimeSinceStartup;
    }

    private void ReceiveServoArmed(BoolMsg message)
    {
        if (message == null)
            return;
        servoArmed = message.data;
        hasServoArmed = true;
        servoArmedTime = Time.realtimeSinceStartup;
    }

    private void ReceiveHardwareStatus(StringMsg message)
    {
        if (message != null && !string.IsNullOrWhiteSpace(message.data))
            hardwareStatus = message.data;
    }

    private void IntegrateEstimatedOdometry(float deltaTime)
    {
        if (!MotorPwmFresh || deltaTime <= 0f || deltaTime > 0.25f)
        {
            estimatedSpeedMetresPerSecond = 0f;
            estimatedSignedSpeedMetresPerSecond = 0f;
            return;
        }

        float leftSpeed = leftPwm / pwmPerMetrePerSecond;
        float rightSpeed = rightPwm / pwmPerMetrePerSecond;
        float linear = (leftSpeed + rightSpeed) * 0.5f;
        float fullDifference = 2f * turnTrackContribution;
        float yawRate = fullDifference > 0.0001f
            ? Mathf.Clamp(
                (leftSpeed - rightSpeed) / fullDifference,
                -1f,
                1f) * simulatedMaximumYawRateDegrees
            : 0f;

        estimatedHeadingDegrees = Mathf.Repeat(
            estimatedHeadingDegrees + yawRate * deltaTime,
            360f);
        float radians = estimatedHeadingDegrees * Mathf.Deg2Rad;
        estimatedDisplacement.x += Mathf.Cos(radians) * linear * deltaTime;
        estimatedDisplacement.y -= Mathf.Sin(radians) * linear * deltaTime;
        estimatedSignedSpeedMetresPerSecond = linear;
        estimatedSpeedMetresPerSecond = Mathf.Abs(linear);
    }

    private bool Fresh(bool hasValue, float timestamp, float timeout)
    {
        return hasValue && Time.realtimeSinceStartup - timestamp <= timeout;
    }

    private float Age(bool hasValue, float timestamp)
    {
        return hasValue
            ? Mathf.Max(0f, Time.realtimeSinceStartup - timestamp)
            : float.PositiveInfinity;
    }

    private static bool Finite(double value)
    {
        return !double.IsNaN(value) && !double.IsInfinity(value);
    }
}
