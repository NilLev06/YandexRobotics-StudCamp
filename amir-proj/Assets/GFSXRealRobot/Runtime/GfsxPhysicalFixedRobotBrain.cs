using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// Physical host for the Yandex GFSX_Brain_Fixed policy.
/// Reproduces the inference-time post-processing that lives outside ONNX:
/// IR-only/automatic close and an immediate drive stop after a confirmed hold.
/// S1-S3 remain a separately armed fixed pose and are not policy actions.
/// </summary>
[DisallowMultipleComponent]
public sealed class GfsxPhysicalFixedRobotBrain : Agent
{
    public const int ObservationSize =
        GfsxPhysicalFixedObservationAdapter.ObservationSize;
    public const int ObservationStackCount =
        GfsxPhysicalFixedObservationAdapter.ObservationStackCount;
    public const int ContinuousActionCount = 3;
    public const int ClawBranchSize = 3;
    public const int RequiredDecisionPeriod = 5;
    private const float SearchAssistStrength = 0.28f;
    private const float SearchAssistSpeedMultiplier = 1.65f;
    private const float SearchAssistConfidenceThreshold = 0.05f;

    [Header("Portable real-robot adapters")]
    [SerializeField]
    private GfsxPhysicalFixedObservationAdapter observations;
    [SerializeField] private GfsxPhysicalActionRouter actionRouter;

    [Header("Policy diagnostics")]
    [SerializeField] private float latestGas;
    [SerializeField] private float latestSteering;
    [SerializeField] private float latestCameraPan;
    [SerializeField] private int latestRawClawCommand;
    [SerializeField] private int latestRoutedClawCommand;
    [SerializeField] private int actionCount;
    [SerializeField] private int blockedEmptyCloseCount;
    [SerializeField] private int automaticIrCloseCount;

    public GfsxPhysicalFixedObservationAdapter Observations => observations;
    public GfsxPhysicalTelemetry Telemetry => observations != null
        ? observations.Telemetry
        : null;
    public GfsxRealVisionReceiver Vision => observations != null
        ? observations.Vision
        : null;
    public GfsxPhysicalActionRouter ActionRouter => actionRouter;
    public int ActionCount => actionCount;
    public float LatestGas => latestGas;
    public float LatestSteering => latestSteering;
    public float LatestCameraPan => latestCameraPan;
    public int LatestRawClawCommand => latestRawClawCommand;
    public int LatestRoutedClawCommand => latestRoutedClawCommand;

    public override void Initialize()
    {
        ResolveReferences();
        actionRouter?.EmergencyStop(
            "Fixed-policy session initialized: all real outputs disarmed.");
    }

    public override void OnEpisodeBegin()
    {
        ResolveReferences();
        observations?.ResetPhysicalState();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        ResolveReferences();
        if (observations != null)
        {
            observations.WriteObservations(sensor);
            return;
        }

        actionRouter?.EmergencyStop(
            "Fixed observation adapter missing: outputs stopped.");
        sensor.AddObservation(1f);
        for (int index = 1; index < ObservationSize; index++)
            sensor.AddObservation(0f);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        ResolveReferences();
        if (actions.ContinuousActions.Length != ContinuousActionCount ||
            actions.DiscreteActions.Length != 1)
        {
            actionRouter?.EmergencyStop(
                "Invalid GFSX_Brain_Fixed action tensor: outputs stopped.");
            enabled = false;
            return;
        }

        latestGas = Sanitize(actions.ContinuousActions[0]);
        latestSteering = Sanitize(actions.ContinuousActions[1]);
        latestCameraPan = Sanitize(actions.ContinuousActions[2]);
        latestRawClawCommand = Mathf.Clamp(
            actions.DiscreteActions[0],
            0,
            ClawBranchSize - 1);
        latestRoutedClawCommand = RouteClawCommand(latestRawClawCommand);
        float effectiveCameraPan = latestCameraPan;
        float cameraPanSpeedMultiplier = 1f;

        // The supplied RobotBrain adds a remembered-target camera sweep
        // outside the ONNX graph. Reproduce it before routing S5.
        bool ballVisible = Vision != null && Vision.BallVisible;
        if (!ballVisible && observations != null &&
            observations.HasBallSightMemory &&
            observations.BallMemoryConfidence >
                SearchAssistConfidenceThreshold &&
            Mathf.Abs(observations.LastKnownBallDirection) > 0.05f)
        {
            effectiveCameraPan = Mathf.Clamp(
                effectiveCameraPan +
                observations.LastKnownBallDirection *
                SearchAssistStrength *
                observations.BallMemoryConfidence,
                -1f,
                1f);
            cameraPanSpeedMultiplier = SearchAssistSpeedMultiplier;
        }

        // The training RobotBrain returns immediately and hard-stops its
        // tracks while GripperController reports a held ball.
        if (actionRouter != null && actionRouter.HasBall)
        {
            latestGas = 0f;
            latestSteering = 0f;
            effectiveCameraPan = 0f;
            cameraPanSpeedMultiplier = 1f;
            latestRoutedClawCommand = 0;
        }

        latestCameraPan = effectiveCameraPan;
        actionCount++;
        actionRouter?.AcceptPolicyAction(
            latestGas,
            latestSteering,
            latestCameraPan,
            cameraPanSpeedMultiplier,
            latestRoutedClawCommand);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuous = actionsOut.ContinuousActions;
        for (int index = 0; index < continuous.Length; index++)
            continuous[index] = 0f;
        ActionSegment<int> discrete = actionsOut.DiscreteActions;
        for (int index = 0; index < discrete.Length; index++)
            discrete[index] = 0;
    }

    public void Configure(
        GfsxPhysicalFixedObservationAdapter observationAdapter,
        GfsxPhysicalActionRouter router)
    {
        observations = observationAdapter;
        actionRouter = router;
    }

    protected override void OnDisable()
    {
        actionRouter?.EmergencyStop(
            "Fixed physical Agent disabled: all real outputs stopped.");
        base.OnDisable();
    }

    private int RouteClawCommand(int requested)
    {
        GfsxPhysicalTelemetry telemetry = Telemetry;
        bool gripperIr =
            telemetry != null && telemetry.SensorFresh &&
            telemetry.GripperIr > 0.5f;

        // Auto-close has priority over every discrete action in the supplied
        // training RobotBrain, including an explicit Open action.
        if (gripperIr)
        {
            if (requested != 1)
                automaticIrCloseCount++;
            return 1;
        }

        // The training policy is not allowed to click an empty gripper.
        if (requested == 1)
        {
            blockedEmptyCloseCount++;
            return 0;
        }
        return requested;
    }

    private void ResolveReferences()
    {
        if (observations == null)
        {
            observations =
                GetComponent<GfsxPhysicalFixedObservationAdapter>();
        }
        if (actionRouter == null)
            actionRouter = GetComponent<GfsxPhysicalActionRouter>();
    }

    private static float Sanitize(float value)
    {
        return float.IsNaN(value) || float.IsInfinity(value)
            ? 0f
            : Mathf.Clamp(value, -1f, 1f);
    }
}
