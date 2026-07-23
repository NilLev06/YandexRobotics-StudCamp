using System;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// Physical observation source for the Yandex GFSX_Brain_Fixed policy.
/// The order, units, fallbacks, and five-second sight-memory decay mirror the
/// exported 2026-07-22 training RobotBrain exactly.
/// </summary>
[DisallowMultipleComponent]
public sealed class GfsxPhysicalFixedObservationAdapter : MonoBehaviour
{
    public const int ObservationSize = 15;
    public const int ObservationStackCount = 4;
    public const float BallMemoryDecaySeconds = 5f;

    private const float BallDirectionDeadband = 0.05f;

    [Header("Real sources only")]
    [SerializeField] private GfsxPhysicalTelemetry telemetry;
    [SerializeField] private GfsxRealVisionReceiver vision;
    [SerializeField] private GfsxPhysicalActionRouter actionRouter;

    [Header("S5 physical-to-training mapping")]
    [SerializeField] private float sensorPanBindDegrees = 90f;
    [SerializeField] private float sensorPanVisualDirection = -1f;
    [SerializeField, Min(1f)] private float sensorPanRelativeRangeDegrees = 75f;

    [Header("Training-contract memory (diagnostic)")]
    [SerializeField] private float lastKnownBallDirection;
    [SerializeField] private float lastKnownBallAngle;
    [SerializeField] private float lastKnownBallDistance = 1f;
    [SerializeField] private float ballMemoryConfidence;
    [SerializeField] private float timeSinceLastDetection;
    [SerializeField] private bool hasBallSightMemory;

    private readonly float[] observations = new float[ObservationSize];

    public GfsxPhysicalTelemetry Telemetry => telemetry;
    public GfsxRealVisionReceiver Vision => vision;
    public GfsxPhysicalActionRouter ActionRouter => actionRouter;
    public float BallMemoryConfidence => ballMemoryConfidence;
    public float TimeSinceLastDetection => timeSinceLastDetection;
    public float LastKnownBallDirection => lastKnownBallDirection;
    public bool HasBallSightMemory => hasBallSightMemory;
    public float SensorPanBindDegrees => sensorPanBindDegrees;
    public float SensorPanVisualDirection => sensorPanVisualDirection;
    public float SensorPanRelativeRangeDegrees =>
        sensorPanRelativeRangeDegrees;

    private void Reset()
    {
        ResolveReferences();
    }

    public void Configure(
        GfsxPhysicalTelemetry physicalTelemetry,
        GfsxRealVisionReceiver realVision,
        GfsxPhysicalActionRouter router)
    {
        telemetry = physicalTelemetry;
        vision = realVision;
        actionRouter = router;
    }

    public void ConfigureSensorPanMapping(
        float bindDegrees,
        float visualDirection,
        float relativeRangeDegrees)
    {
        sensorPanBindDegrees = bindDegrees;
        sensorPanVisualDirection = visualDirection < 0f ? -1f : 1f;
        sensorPanRelativeRangeDegrees = Mathf.Max(
            1f, relativeRangeDegrees);
    }

    public void WriteObservations(VectorSensor sensor)
    {
        if (sensor == null)
            throw new ArgumentNullException(nameof(sensor));

        FillObservations(observations);
        for (int index = 0; index < ObservationSize; index++)
            sensor.AddObservation(observations[index]);
    }

    public int FillObservations(float[] destination, int offset = 0)
    {
        if (destination == null)
            throw new ArgumentNullException(nameof(destination));
        if (offset < 0 || destination.Length - offset < ObservationSize)
            throw new ArgumentException(
                "Destination must have room for 15 Fixed-policy observations.",
                nameof(destination));

        ResolveReferences();
        UpdateBallSightMemory();

        bool sensorsFresh = telemetry != null && telemetry.SensorFresh;
        bool visible = vision != null && vision.BallVisible;
        Vector2 displacement = telemetry != null
            ? telemetry.EstimatedDisplacement
            : Vector2.zero;
        float headingDegrees = telemetry != null
            ? telemetry.EstimatedHeadingDegrees
            : 0f;
        if (headingDegrees > 180f)
            headingDegrees -= 360f;

        // Do not normalize these values again. These exact units are what the
        // exported Fixed policy's running normalizer was trained to receive.
        destination[offset + 0] = sensorsFresh
            ? telemetry.UltrasonicNormalized
            : 1f;
        destination[offset + 1] = sensorsFresh ? telemetry.LeftIr : 0f;
        destination[offset + 2] = sensorsFresh ? telemetry.RightIr : 0f;
        destination[offset + 3] = sensorsFresh ? telemetry.GripperIr : 0f;
        destination[offset + 4] = visible
            ? vision.NormalizedAngle
            : lastKnownBallAngle * ballMemoryConfidence;
        destination[offset + 5] = visible
            ? vision.NormalizedDistance
            : Mathf.Lerp(
                1f,
                lastKnownBallDistance,
                ballMemoryConfidence);
        destination[offset + 6] =
            lastKnownBallDirection * ballMemoryConfidence;
        destination[offset + 7] = visible ? 1f : 0f;
        float physicalPanDegrees;
        destination[offset + 8] =
            telemetry != null &&
            telemetry.TryGetSensorPanDegrees(out physicalPanDegrees)
                ? Mathf.Clamp(
                    (physicalPanDegrees - sensorPanBindDegrees) *
                    sensorPanVisualDirection /
                    sensorPanRelativeRangeDegrees,
                    -1f,
                    1f)
                : 0f;
        destination[offset + 9] =
            actionRouter != null && actionRouter.HasBall ? 1f : 0f;
        destination[offset + 10] = displacement.x;
        destination[offset + 11] = displacement.y;
        destination[offset + 12] =
            Mathf.Clamp(headingDegrees / 180f, -1f, 1f);
        destination[offset + 13] = telemetry != null
            ? telemetry.EstimatedSignedSpeedMetresPerSecond
            : 0f;
        destination[offset + 14] = Mathf.Clamp01(
            timeSinceLastDetection / BallMemoryDecaySeconds);

        return ObservationSize;
    }

    public void ResetPhysicalState()
    {
        ResolveReferences();
        lastKnownBallDirection = 0f;
        lastKnownBallAngle = 0f;
        lastKnownBallDistance = 1f;
        ballMemoryConfidence = 0f;
        timeSinceLastDetection = 0f;
        hasBallSightMemory = false;
        telemetry?.ResetDeadReckoning();
        vision?.ResetDetectionMemory();
        actionRouter?.StopPolicyAction();
    }

    private void UpdateBallSightMemory()
    {
        if (vision == null)
            return;

        if (vision.BallVisible)
        {
            hasBallSightMemory = true;
            lastKnownBallAngle = vision.NormalizedAngle;
            lastKnownBallDistance = vision.NormalizedDistance;
            lastKnownBallDirection =
                Mathf.Abs(lastKnownBallAngle) < BallDirectionDeadband
                    ? 0f
                    : Mathf.Sign(lastKnownBallAngle);
            ballMemoryConfidence = 1f;
            timeSinceLastDetection = 0f;
            return;
        }

        // This intentionally advances once per CollectObservations call, just
        // like the supplied training RobotBrain (DecisionPeriod = 5).
        timeSinceLastDetection += Time.fixedDeltaTime;
        if (!hasBallSightMemory)
        {
            ballMemoryConfidence = 0f;
            return;
        }

        ballMemoryConfidence = Mathf.Clamp01(
            1f - timeSinceLastDetection / BallMemoryDecaySeconds);
        if (ballMemoryConfidence <= 0.001f)
        {
            hasBallSightMemory = false;
            lastKnownBallDirection = 0f;
            lastKnownBallAngle = 0f;
            lastKnownBallDistance = 1f;
        }
    }

    private void ResolveReferences()
    {
        if (telemetry == null)
            telemetry = GetComponent<GfsxPhysicalTelemetry>();
        if (vision == null)
            vision = GetComponent<GfsxRealVisionReceiver>();
        if (actionRouter == null)
            actionRouter = GetComponent<GfsxPhysicalActionRouter>();
    }
}
