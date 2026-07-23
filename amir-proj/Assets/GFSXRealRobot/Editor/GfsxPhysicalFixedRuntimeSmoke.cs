using System;
using Unity.Robotics.ROSTCPConnector;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>
/// Executes the exact Fixed ONNX for several decisions while the router is
/// explicitly network-silent and every physical output remains disarmed.
/// </summary>
[InitializeOnLoad]
public static class GfsxPhysicalFixedRuntimeSmoke
{
    private const string RunningKey = "GFSX.FixedPhysicalSmoke.Running";
    private const string CommandLineKey =
        "GFSX.FixedPhysicalSmoke.CommandLine";
    private const string ResultKey = "GFSX.FixedPhysicalSmoke.Result";
    private const int RequiredActionCount = 5;
    private const double TimeoutSeconds = 35.0;
    private static double playStartedAt;
    private static bool hooked;

    static GfsxPhysicalFixedRuntimeSmoke()
    {
        if (SessionState.GetBool(RunningKey, false))
        {
            Hook();
            EditorApplication.delayCall += ResumeAfterDomainReload;
        }
    }

    [MenuItem("GFS-X/ML Agents/Run FIXED Real Robot NETWORK-SILENT Smoke Test")]
    public static void RunFromMenu()
    {
        Begin(false);
    }

    public static void RunFromCommandLine()
    {
        Begin(true);
    }

    private static void Begin(bool exitEditorWhenDone)
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
            throw new InvalidOperationException("Exit Play Mode first.");

        GfsxPhysicalFixedLiveSetup.ValidateModelHash();
        EditorSceneManager.OpenScene(
            GfsxPhysicalFixedLiveSetup.ScenePath,
            OpenSceneMode.Single);

        GfsxPhysicalFixedPreflightGuard guard =
            UnityEngine.Object.FindFirstObjectByType<
                GfsxPhysicalFixedPreflightGuard>();
        GfsxPhysicalActionRouter router =
            UnityEngine.Object.FindFirstObjectByType<
                GfsxPhysicalActionRouter>();
        GfsxPhysicalTelemetry telemetry =
            UnityEngine.Object.FindFirstObjectByType<
                GfsxPhysicalTelemetry>();
        if (guard == null || router == null || telemetry == null ||
            !guard.ValidateForLaunch(true))
        {
            throw new InvalidOperationException(
                guard != null
                    ? guard.LastReport
                    : "Fixed physical runtime components are missing.");
        }

        // These changes are intentionally not saved. They guarantee the
        // smoke process cannot register physical ROS publishers and cannot
        // connect its read-only subscriber to the real robot.
        telemetry.Configure("127.0.0.1", 10000);
        router.ConfigurePhysicalPublishingSuppressed(true);

        SessionState.SetBool(RunningKey, true);
        SessionState.SetBool(CommandLineKey, exitEditorWhenDone);
        SessionState.SetInt(ResultKey, 0);
        playStartedAt = 0.0;
        Hook();
        Debug.Log(
            "GFS-X Fixed smoke: entering network-silent inference. " +
            "All physical outputs are disarmed and ROS publishing is suppressed.");
        EditorApplication.isPlaying = true;
    }

    private static void Hook()
    {
        if (hooked)
            return;
        hooked = true;
        EditorApplication.update += OnEditorUpdate;
        EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
    }

    private static void Unhook()
    {
        if (!hooked)
            return;
        hooked = false;
        EditorApplication.update -= OnEditorUpdate;
        EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
    }

    private static void ResumeAfterDomainReload()
    {
        if (!SessionState.GetBool(RunningKey, false))
            return;
        if (!EditorApplication.isPlaying &&
            !EditorApplication.isPlayingOrWillChangePlaymode &&
            SessionState.GetInt(ResultKey, 0) != 0)
        {
            FinishInEditMode();
        }
    }

    private static void OnPlayModeStateChanged(PlayModeStateChange state)
    {
        if (!SessionState.GetBool(RunningKey, false))
            return;
        if (state == PlayModeStateChange.EnteredPlayMode)
            playStartedAt = EditorApplication.timeSinceStartup;
        else if (state == PlayModeStateChange.EnteredEditMode &&
                 SessionState.GetInt(ResultKey, 0) != 0)
            FinishInEditMode();
    }

    private static void OnEditorUpdate()
    {
        if (!SessionState.GetBool(RunningKey, false) ||
            !EditorApplication.isPlaying)
            return;
        if (playStartedAt <= 0.0)
            playStartedAt = EditorApplication.timeSinceStartup;

        try
        {
            GfsxPhysicalFixedPreflightGuard guard =
                UnityEngine.Object.FindFirstObjectByType<
                    GfsxPhysicalFixedPreflightGuard>();
            GfsxPhysicalFixedRobotBrain brain =
                UnityEngine.Object.FindFirstObjectByType<
                    GfsxPhysicalFixedRobotBrain>();
            GfsxPhysicalFixedObservationAdapter observations =
                UnityEngine.Object.FindFirstObjectByType<
                    GfsxPhysicalFixedObservationAdapter>();
            GfsxPhysicalActionRouter router =
                UnityEngine.Object.FindFirstObjectByType<
                    GfsxPhysicalActionRouter>();

            if (guard == null || brain == null ||
                observations == null || router == null)
            {
                Fail("A Fixed physical runtime component is missing.");
                return;
            }
            if (!guard.ReadyForFixedLiveInference || !brain.enabled)
            {
                Fail("Fixed preflight blocked the Agent: " + guard.LastReport);
                return;
            }
            if (!router.PhysicalPublishingSuppressed)
            {
                Fail("The network-silent router flag was not preserved.");
                return;
            }
            if (router.DriveArmed || router.CameraServosArmed ||
                router.ClawServoArmed || router.FixedArmPoseArmed)
            {
                Fail("A real output became armed during the dry smoke.");
                return;
            }
            if (HasPhysicalOutputPublisher())
            {
                Fail("A physical ROS output topic was registered.");
                return;
            }
            if (!Finite(brain.LatestGas) ||
                !Finite(brain.LatestSteering) ||
                !Finite(brain.LatestCameraPan) ||
                brain.LatestRoutedClawCommand < 0 ||
                brain.LatestRoutedClawCommand > 2)
            {
                Fail("The Fixed ONNX produced an invalid action.");
                return;
            }

            if (brain.ActionCount >= RequiredActionCount)
            {
                if (router.ReceivedActionCount < RequiredActionCount)
                {
                    Fail("The router did not receive all policy decisions.");
                    return;
                }
                Debug.Log(
                    "GFS-X FIXED NETWORK-SILENT SMOKE PASS: exact ONNX " +
                    "executed " + brain.ActionCount +
                    " decisions; no physical publisher existed and all " +
                    "drive/servo channels stayed disarmed.");
                Complete(1);
                return;
            }

            if (EditorApplication.timeSinceStartup - playStartedAt >
                TimeoutSeconds)
            {
                Fail("Timed out with actions=" + brain.ActionCount + ".");
            }
        }
        catch (Exception exception)
        {
            Fail(exception.ToString());
        }
    }

    private static bool Finite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }

    private static bool HasPhysicalOutputPublisher()
    {
        ROSConnection connection =
            UnityEngine.Object.FindFirstObjectByType<ROSConnection>();
        if (connection == null)
            return false;
        string[] topics =
        {
            "/gfsx/cmd_vel",
            "/gfsx/drive_enable",
            "/gfsx/servo_targets_degrees",
            "/gfsx/servo_enable"
        };
        foreach (string topic in topics)
        {
            RosTopicState state = connection.GetTopic(topic);
            if (state != null && state.IsPublisher)
                return true;
        }
        return false;
    }

    private static void Fail(string reason)
    {
        Debug.LogError(
            "GFS-X FIXED NETWORK-SILENT SMOKE FAIL: " + reason);
        Complete(-1);
    }

    private static void Complete(int result)
    {
        SessionState.SetInt(ResultKey, result);
        GfsxPhysicalActionRouter router =
            UnityEngine.Object.FindFirstObjectByType<
                GfsxPhysicalActionRouter>();
        router?.EmergencyStop(
            "Fixed smoke complete: all physical output remains stopped.");
        if (EditorApplication.isPlaying)
            EditorApplication.isPlaying = false;
        else
            FinishInEditMode();
    }

    private static void FinishInEditMode()
    {
        GfsxPhysicalTelemetry telemetry =
            UnityEngine.Object.FindFirstObjectByType<
                GfsxPhysicalTelemetry>();
        GfsxPhysicalActionRouter router =
            UnityEngine.Object.FindFirstObjectByType<
                GfsxPhysicalActionRouter>();
        telemetry?.Configure("192.168.2.152", 10000);
        router?.ConfigurePhysicalPublishingSuppressed(false);

        bool commandLine = SessionState.GetBool(CommandLineKey, false);
        int result = SessionState.GetInt(ResultKey, -1);
        SessionState.EraseBool(RunningKey);
        SessionState.EraseBool(CommandLineKey);
        SessionState.EraseInt(ResultKey);
        Unhook();
        if (commandLine)
            EditorApplication.Exit(result > 0 ? 0 : 1);
    }
}
