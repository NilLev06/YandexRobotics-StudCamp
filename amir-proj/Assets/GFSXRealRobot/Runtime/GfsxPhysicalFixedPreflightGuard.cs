using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Policies;
using UnityEngine;

/// <summary>
/// Static launch validation for the dedicated GFSX_Brain_Fixed real-robot
/// scene. Runtime freshness, obstacle, watchdog, and two-step arming gates
/// remain enforced continuously by GfsxPhysicalActionRouter.
/// </summary>
[DefaultExecutionOrder(-1000)]
[DisallowMultipleComponent]
public sealed class GfsxPhysicalFixedPreflightGuard : MonoBehaviour
{
    public const float ExpectedFixedS1Degrees = 70f;
    public const float ExpectedFixedS2Degrees = 180f;
    public const float ExpectedFixedS3Degrees = 90f;
    public const float ExpectedFixedS6Degrees = 75f;

    [SerializeField] private GfsxPhysicalFixedRobotBrain brain;
    [SerializeField] private BehaviorParameters behavior;
    [SerializeField] private DecisionRequester requester;
    [SerializeField] private GfsxPhysicalTelemetry telemetry;
    [SerializeField] private GfsxRealVisionReceiver vision;
    [SerializeField] private GfsxPhysicalActionRouter router;
    [SerializeField]
    private GfsxPhysicalFixedObservationAdapter observations;
    [SerializeField, TextArea(7, 20)] private string lastReport;
    [SerializeField] private bool readyForFixedLiveInference;

    public string LastReport => lastReport;
    public bool ReadyForFixedLiveInference => readyForFixedLiveInference;

    private void Awake()
    {
        ValidateForLaunch(true);
    }

    public void Configure(
        GfsxPhysicalFixedRobotBrain physicalBrain,
        BehaviorParameters behaviorParameters,
        DecisionRequester decisionRequester,
        GfsxPhysicalTelemetry physicalTelemetry,
        GfsxRealVisionReceiver realVision,
        GfsxPhysicalActionRouter actionRouter,
        GfsxPhysicalFixedObservationAdapter observationAdapter)
    {
        brain = physicalBrain;
        behavior = behaviorParameters;
        requester = decisionRequester;
        telemetry = physicalTelemetry;
        vision = realVision;
        router = actionRouter;
        observations = observationAdapter;
    }

    public bool ValidateForLaunch(bool logResult)
    {
        ResolveReferences();
        List<string> errors = new List<string>();
        List<string> warnings = new List<string>();

        if (brain == null || behavior == null || requester == null ||
            telemetry == null || vision == null || router == null ||
            observations == null)
        {
            errors.Add(
                "One or more Fixed-policy physical components are missing.");
        }

        if (behavior != null)
        {
            if (behavior.BehaviorName != "GFSX_Brain_Fixed")
                errors.Add("Behavior Name must be GFSX_Brain_Fixed.");
            if (behavior.BehaviorType != BehaviorType.InferenceOnly)
                errors.Add("Behavior Type must be Inference Only.");
            if (behavior.Model == null)
                errors.Add("GFSX_Brain_Fixed.onnx is not assigned.");
            else if (behavior.Model.name != "GFSX_Brain_Fixed")
                errors.Add("Assigned model must be GFSX_Brain_Fixed.onnx.");
            if (!behavior.DeterministicInference)
                errors.Add("Deterministic Inference must be enabled.");
            if (behavior.BrainParameters.VectorObservationSize !=
                GfsxPhysicalFixedRobotBrain.ObservationSize)
                errors.Add("Vector observation size must be 15.");
            if (behavior.BrainParameters.NumStackedVectorObservations !=
                GfsxPhysicalFixedRobotBrain.ObservationStackCount)
                errors.Add("Vector observations must be stacked four times.");
            if (behavior.BrainParameters.ActionSpec.NumContinuousActions !=
                GfsxPhysicalFixedRobotBrain.ContinuousActionCount)
                errors.Add("The Fixed model requires three continuous actions.");
            int[] branches =
                behavior.BrainParameters.ActionSpec.BranchSizes;
            if (branches == null || branches.Length != 1 ||
                branches[0] != GfsxPhysicalFixedRobotBrain.ClawBranchSize)
                errors.Add(
                    "The Fixed model requires one discrete branch of size three.");
        }

        if (requester != null &&
            (requester.DecisionPeriod !=
                 GfsxPhysicalFixedRobotBrain.RequiredDecisionPeriod ||
             !requester.TakeActionsBetweenDecisions))
        {
            errors.Add(
                "DecisionRequester must use period 5 and repeat actions.");
        }
        if (brain != null && brain.MaxStep != 0)
            errors.Add("Physical inference Agent MaxStep must be zero.");

        Agent[] agents = Object.FindObjectsByType<Agent>(
            FindObjectsInactive.Include,
            FindObjectsSortMode.None);
        if (agents.Length != 1 || agents[0] != brain)
            errors.Add("The Fixed LIVE scene must contain exactly one Agent.");

        if (HasForbiddenComponent())
        {
            errors.Add(
                "Simulation, legacy ROS, Mobile, or old physical policy " +
                "components must not exist on the Fixed Agent host.");
        }

        if (router != null)
        {
            if (!router.AllowDriveArming)
                errors.Add("Fixed LIVE scene must make two-step track arming available.");
            if (!router.AllowClawServoActions)
                errors.Add("Fixed LIVE scene must make two-step S4 arming available.");
            if (!router.AllowFixedArmPose)
                errors.Add(
                    "Fixed LIVE scene must make two-step S1-S3 pose arming available.");
            if (!router.RequireFixedArmPoseBeforeDrive)
                errors.Add(
                    "Tracks must remain blocked until S1-S3 is armed and at target.");
            if (!router.FixedArmTargetsWithinLimits)
                errors.Add(
                    "Fixed S1-S3 targets must remain within 0-180 degrees.");
            if (!Approximately(
                    router.FixedArmS1Degrees,
                    ExpectedFixedS1Degrees) ||
                !Approximately(
                    router.FixedArmS2Degrees,
                    ExpectedFixedS2Degrees) ||
                !Approximately(
                    router.FixedArmS3Degrees,
                    ExpectedFixedS3Degrees) ||
                !Approximately(
                    router.FixedCameraTiltDegrees,
                    ExpectedFixedS6Degrees))
            {
                errors.Add(
                    "Fixed pose must be the manually calibrated physical " +
                    "S1/S2/S3 = 70/180/90 and training-equivalent S6 = 75 degrees.");
            }
            if (!Approximately(router.SensorPanDegreesPerSecond, 60f))
                errors.Add("Fixed S5 policy rate must be 60 degrees/second.");
            if (!Approximately(router.SensorPanActionDirection, -1f))
                errors.Add(
                    "Fixed S5 action direction must be -1 for the physical calibration.");
        }

        if (observations != null)
        {
            if (!Approximately(observations.SensorPanBindDegrees, 90f) ||
                !Approximately(
                    observations.SensorPanVisualDirection, -1f) ||
                !Approximately(
                    observations.SensorPanRelativeRangeDegrees, 75f))
            {
                errors.Add(
                    "Fixed observation 8 requires S5 bind/direction/range = 90/-1/75.");
            }
        }

        warnings.Add(
            "The robot has no wheel encoders or IMU; observations 10-13 use " +
            "motor-PWM dead reckoning rather than measured world pose.");
        warnings.Add(
            "The supplied 15,008,000-step Fixed model recorded zero catches " +
            "and never observed holding=1. Treat it as an experimental " +
            "search/drive policy, not a validated grasp policy.");
        warnings.Add(
            "Every Play session starts disarmed. F6 arms fixed S1-S3/S6, F8 " +
            "arms S5, F7 arms S4, and F9 arms tracks; each requires two " +
            "presses. F10/Backspace stops everything.");

        readyForFixedLiveInference = errors.Count == 0;
        lastReport = (readyForFixedLiveInference
            ? "READY FOR GFSX_BRAIN_FIXED PHYSICAL LIVE INFERENCE."
            : "NOT READY FOR GFSX_BRAIN_FIXED PHYSICAL LIVE INFERENCE.") +
            (errors.Count > 0
                ? "\nERRORS:\n- " + string.Join("\n- ", errors)
                : string.Empty) +
            "\nWARNINGS:\n- " + string.Join("\n- ", warnings);

        if (!readyForFixedLiveInference)
        {
            router?.EmergencyStop(
                "Fixed-policy preflight failed: outputs stopped.");
            if (brain != null && Application.isPlaying)
                brain.enabled = false;
        }

        if (logResult)
        {
            if (readyForFixedLiveInference)
                Debug.Log(lastReport, this);
            else
                Debug.LogError(lastReport, this);
        }
        return readyForFixedLiveInference;
    }

    private void ResolveReferences()
    {
        if (brain == null)
            brain = GetComponent<GfsxPhysicalFixedRobotBrain>();
        if (behavior == null)
            behavior = GetComponent<BehaviorParameters>();
        if (requester == null)
            requester = GetComponent<DecisionRequester>();
        if (telemetry == null)
            telemetry = GetComponent<GfsxPhysicalTelemetry>();
        if (vision == null)
            vision = GetComponent<GfsxRealVisionReceiver>();
        if (router == null)
            router = GetComponent<GfsxPhysicalActionRouter>();
        if (observations == null)
        {
            observations =
                GetComponent<GfsxPhysicalFixedObservationAdapter>();
        }
    }

    private bool HasForbiddenComponent()
    {
        HashSet<string> forbiddenTypeNames = new HashSet<string>
        {
            "RobotBrain",
            "VirtualSensors",
            "SimulatedYoloCamera",
            "TrackController",
            "GfsxRosTcpController",
            "GfsxPhysicalRobotBrain",
            "GfsxPhysicalObservationAdapter",
            "GfsxPhysicalPreflightGuard",
            "GfsxPhysicalMobileShadowBrain",
            "GfsxPhysicalMobileShadowGuard"
        };
        foreach (MonoBehaviour component in GetComponents<MonoBehaviour>())
        {
            if (component != null &&
                forbiddenTypeNames.Contains(component.GetType().Name))
                return true;
        }
        return false;
    }

    private static bool Approximately(float value, float expected)
    {
        return Mathf.Abs(value - expected) <= 0.01f;
    }
}
