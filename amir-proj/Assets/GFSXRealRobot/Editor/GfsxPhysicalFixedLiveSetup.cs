using System;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using Unity.InferenceEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Creates a dedicated physical-live scene for the Yandex Fixed-arm model.
/// Existing Mobile, legacy, training, and physical scenes are not modified.
/// </summary>
public static class GfsxPhysicalFixedLiveSetup
{
    public const string ScenePath =
        "Assets/GFSXRealRobot/Scenes/P4_RealRobotFixedLive.unity";
    public const string ModelPath =
        "Assets/GFSXRealRobot/Models/GFSX_Brain_Fixed.onnx";
    public const string ExpectedModelSha256 =
        "28919e156d31cd9d26ad22bde3a338b878ab0a9ea3dfde5557148ec0bbace488";
    private const string HostName =
        "GFS-X Fixed Physical Agent (LIVE)";

    [MenuItem("GFS-X/ML Agents/Create or Repair FIXED Real Robot LIVE Scene")]
    public static void CreateOrRepairFixedLiveScene()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            throw new InvalidOperationException(
                "Exit Play mode before creating the Fixed LIVE scene.");
        }

        ValidateModelHash();
        AssetDatabase.ImportAsset(ModelPath, ImportAssetOptions.ForceUpdate);

        Scene scene = File.Exists(ScenePath)
            ? EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single)
            : EditorSceneManager.NewScene(
                NewSceneSetup.EmptyScene,
                NewSceneMode.Single);

        ModelAsset model =
            AssetDatabase.LoadAssetAtPath<ModelAsset>(ModelPath);
        if (model == null)
        {
            throw new FileNotFoundException(
                "GFSX_Brain_Fixed.onnx did not import as a Unity ModelAsset.",
                ModelPath);
        }

        GameObject host = FindRoot(scene, HostName);
        if (host == null)
            host = new GameObject(HostName);
        GameObjectUtility.RemoveMonoBehavioursWithMissingScript(host);
        RemoveIncompatibleComponents(host);

        GfsxPhysicalTelemetry telemetry =
            GetOrAdd<GfsxPhysicalTelemetry>(host);
        GfsxRealVisionReceiver vision =
            GetOrAdd<GfsxRealVisionReceiver>(host);
        GfsxPhysicalActionRouter router =
            GetOrAdd<GfsxPhysicalActionRouter>(host);
        GfsxPhysicalFixedObservationAdapter observations =
            GetOrAdd<GfsxPhysicalFixedObservationAdapter>(host);
        GfsxPhysicalFixedRobotBrain brain =
            GetOrAdd<GfsxPhysicalFixedRobotBrain>(host);
        BehaviorParameters behavior =
            GetOrAdd<BehaviorParameters>(host);
        DecisionRequester requester =
            GetOrAdd<DecisionRequester>(host);
        GfsxPhysicalFixedPreflightGuard guard =
            GetOrAdd<GfsxPhysicalFixedPreflightGuard>(host);

        behavior.BehaviorName = "GFSX_Brain_Fixed";
        behavior.BehaviorType = BehaviorType.InferenceOnly;
        behavior.Model = model;
        behavior.DeterministicInference = true;
        behavior.UseChildSensors = false;
        behavior.UseChildActuators = false;
        behavior.TeamId = 0;
        behavior.BrainParameters.VectorObservationSize =
            GfsxPhysicalFixedRobotBrain.ObservationSize;
        behavior.BrainParameters.NumStackedVectorObservations =
            GfsxPhysicalFixedRobotBrain.ObservationStackCount;
        behavior.BrainParameters.ActionSpec = new ActionSpec(
            GfsxPhysicalFixedRobotBrain.ContinuousActionCount,
            new[] { GfsxPhysicalFixedRobotBrain.ClawBranchSize });
        behavior.BrainParameters.VectorActionDescriptions = new[]
        {
            "Physical gas (+ forward; capped by router)",
            "Physical steering (+ right; differential tracks)",
            "Physical S5 pan velocity (+ visual right; reversed servo degrees)",
            "Physical S4: 0 preserve / 1 IR-gated close / 2 open"
        };

        requester.DecisionPeriod =
            GfsxPhysicalFixedRobotBrain.RequiredDecisionPeriod;
        requester.DecisionStep = 0;
        requester.TakeActionsBetweenDecisions = true;
        brain.MaxStep = 0;

        telemetry.Configure("192.168.2.152", 10000);
        telemetry.ConfigureFreshness(
            sensorSeconds: 1.0f,
            motorPwmSeconds: 0.75f,
            servoSeconds: 2.0f);
        vision.Configure(5005, 1.0f);

        router.Configure(telemetry, vision);
        router.ConfigurePhysicalPublishingSuppressed(false);
        router.ConfigureOutputPermissions(
            allowDrive: true,
            allowCameraPan: true,
            allowClaw: true);
        router.ConfigureLiveSafety(
            linearLimit: 0.12f,
            trackPwmLimit: 35f,
            watchdogSeconds: 0.45f,
            requireRearIr: true);
        router.ConfigureServoLimits(
            panMinimumDegrees: 15f,
            panMaximumDegrees: 160f,
            panDegreesPerSecond: 60f,
            openClawDegrees: 0f,
            closedClawDegrees: 50f,
            clawCooldownSeconds: 0.5f,
            panActionDirection: -1f);
        router.ConfigureFixedArmPose(
            allow: true,
            requireBeforeDrive: true,
            s1Degrees: 70f,
            s2Degrees: 180f,
            s3Degrees: 90f,
            toleranceDegrees: 1f,
            s6TiltDegrees: 75f);

        observations.Configure(telemetry, vision, router);
        observations.ConfigureSensorPanMapping(
            bindDegrees: 90f,
            visualDirection: -1f,
            relativeRangeDegrees: 75f);
        brain.Configure(observations, router);
        guard.Configure(
            brain,
            behavior,
            requester,
            telemetry,
            vision,
            router,
            observations);

        foreach (UnityEngine.Object item in new UnityEngine.Object[]
        {
            host, telemetry, vision, router, observations, brain, behavior,
            requester, guard
        })
        {
            EditorUtility.SetDirty(item);
        }

        Directory.CreateDirectory(Path.GetDirectoryName(ScenePath));
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene, ScenePath))
            throw new IOException("Could not save " + ScenePath);

        if (!guard.ValidateForLaunch(true))
            throw new InvalidOperationException(guard.LastReport);

        EditorUtility.SetDirty(guard);
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        AddSceneToBuildSettings(ScenePath);
        AssetDatabase.SaveAssets();
        Selection.activeGameObject = host;
        Debug.Log(
            "Created Fixed real-robot LIVE scene at " + ScenePath +
            ". Every Play starts fully disarmed. F6 authorizes S1-S3/S6, " +
            "F8 S5, F7 S4, F9 tracks; each requires two confirmations. " +
            "F10 or Backspace stops all physical output.",
            host);
    }

    public static void CreateFixedLiveSceneForAutomation()
    {
        CreateOrRepairFixedLiveScene();
    }

    [MenuItem("GFS-X/ML Agents/Validate FIXED Real Robot LIVE Scene")]
    public static void ValidateFixedLiveScene()
    {
        ValidateModelHash();
        GfsxPhysicalFixedPreflightGuard guard =
            UnityEngine.Object.FindFirstObjectByType<
                GfsxPhysicalFixedPreflightGuard>();
        if (guard == null)
            throw new InvalidOperationException(
                "Open P4_RealRobotFixedLive first.");
        if (!guard.ValidateForLaunch(true))
            throw new InvalidOperationException(guard.LastReport);
    }

    public static void ValidateModelHash()
    {
        string fullPath = Path.GetFullPath(ModelPath);
        if (!File.Exists(fullPath))
        {
            throw new FileNotFoundException(
                "The exact GFSX_Brain_Fixed ONNX model is missing.",
                fullPath);
        }

        string actual;
        using (SHA256 sha256 = SHA256.Create())
        using (FileStream stream = File.OpenRead(fullPath))
        {
            actual = BitConverter.ToString(sha256.ComputeHash(stream))
                .Replace("-", string.Empty)
                .ToLowerInvariant();
        }

        if (!string.Equals(
                actual,
                ExpectedModelSha256,
                StringComparison.OrdinalIgnoreCase))
        {
            throw new InvalidDataException(
                "GFSX_Brain_Fixed.onnx SHA-256 mismatch. Expected " +
                ExpectedModelSha256 + ", received " + actual + ".");
        }
    }

    private static T GetOrAdd<T>(GameObject host) where T : Component
    {
        T component = host.GetComponent<T>();
        return component != null ? component : host.AddComponent<T>();
    }

    private static GameObject FindRoot(Scene scene, string name)
    {
        foreach (GameObject root in scene.GetRootGameObjects())
        {
            if (root.name == name)
                return root;
        }
        return null;
    }

    private static void RemoveIncompatibleComponents(GameObject host)
    {
        HashSet<string> incompatibleTypeNames = new HashSet<string>
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
        foreach (MonoBehaviour component in host.GetComponents<MonoBehaviour>())
        {
            if (component != null &&
                incompatibleTypeNames.Contains(component.GetType().Name))
            {
                UnityEngine.Object.DestroyImmediate(component);
            }
        }
    }

    private static void AddSceneToBuildSettings(string path)
    {
        List<EditorBuildSettingsScene> scenes =
            new List<EditorBuildSettingsScene>(EditorBuildSettings.scenes);
        int existing = scenes.FindIndex(item => item.path == path);
        if (existing >= 0)
            scenes[existing] = new EditorBuildSettingsScene(path, true);
        else
            scenes.Add(new EditorBuildSettingsScene(path, true));
        EditorBuildSettings.scenes = scenes.ToArray();
    }
}
