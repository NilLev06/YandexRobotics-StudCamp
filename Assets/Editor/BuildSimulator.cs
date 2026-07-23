using System.IO;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class BuildSimulator
{
    private static readonly string[] Scenes =
    {
        "Assets/Scenes/P3_DigitalTwin_FixedArm.unity"
    };
    private const string LinuxBuildPath = "Build/GFSX_Simulator";
    private const string LinuxDataFolder = "Build/GFSX_Simulator_Data";

    public static void BuildLinux()
    {
        // Visual Validation can leave P2 in InferenceOnly — force training mode before build.
        ForceTrainingBehaviorOnScenes();

        BuildPlayerOptions options = new BuildPlayerOptions
        {
            scenes = Scenes,
            locationPathName = LinuxBuildPath,
            target = BuildTarget.StandaloneLinux64,
            options = BuildOptions.None
        };

        BuildReport report = BuildPipeline.BuildPlayer(options);
        if (report.summary.result != BuildResult.Succeeded)
        {
            Debug.LogError($"Build failed: {report.summary.result}");
            EditorApplication.Exit(1);
            return;
        }

        try
        {
            InstallGrpcNativeLibraries();
        }
        catch (IOException exception)
        {
            Debug.LogError($"Failed to install gRPC native libraries: {exception.Message}");
            EditorApplication.Exit(1);
            return;
        }

        Debug.Log($"Build succeeded: {LinuxBuildPath}");
        EditorApplication.Exit(0);
    }

    private static void ForceTrainingBehaviorOnScenes()
    {
        for (int sceneIndex = 0; sceneIndex < Scenes.Length; sceneIndex++)
        {
            string scenePath = Scenes[sceneIndex];
            Scene scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
            if (!scene.IsValid())
                continue;

            RobotBrain[] agents = Object.FindObjectsByType<RobotBrain>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);
            for (int agentIndex = 0; agentIndex < agents.Length; agentIndex++)
            {
                RobotBrain agent = agents[agentIndex];
                BehaviorParameters behavior = agent.GetComponent<BehaviorParameters>();
                if (behavior != null)
                {
                    behavior.BehaviorType = BehaviorType.Default;
                    behavior.Model = null;
                    EditorUtility.SetDirty(behavior);
                }

                EditorUtility.SetDirty(agent);
            }

            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
            Debug.Log($"BuildSimulator: forced training BehaviorType=Default on {scenePath}");
        }
    }

    private static string FindGrpcSourcePath(string projectRoot)
    {
        string packageCache = Path.Combine(projectRoot, "Library", "PackageCache");
        if (!Directory.Exists(packageCache))
        {
            throw new DirectoryNotFoundException($"Package cache not found: {packageCache}");
        }

        string[] matches = Directory.GetDirectories(
            packageCache,
            "com.unity.ml-agents@*",
            SearchOption.TopDirectoryOnly);
        if (matches.Length == 0)
        {
            throw new DirectoryNotFoundException("ML-Agents package not found in PackageCache.");
        }

        string grpcSource = Path.Combine(
            matches[0],
            "Plugins",
            "ProtoBuffer",
            "runtimes",
            "linux",
            "native",
            "libgrpc_csharp_ext.x64.so");
        if (!File.Exists(grpcSource))
        {
            throw new FileNotFoundException("gRPC native library not found.", grpcSource);
        }

        return grpcSource;
    }

    private static void InstallGrpcNativeLibraries()
    {
        string projectRoot = Directory.GetParent(Application.dataPath).FullName;
        string grpcSource = FindGrpcSourcePath(projectRoot);
        string[] destinationDirectories =
        {
            Path.Combine(LinuxDataFolder, "Managed"),
            Path.Combine(LinuxDataFolder, "Plugins"),
            Path.Combine(LinuxDataFolder, "Plugins", "x86_64"),
            Path.Combine(LinuxDataFolder, "Plugins", "AnyCPU")
        };

        foreach (string destinationDirectory in destinationDirectories)
        {
            Directory.CreateDirectory(destinationDirectory);
            string destinationPath = Path.Combine(
                destinationDirectory,
                "libgrpc_csharp_ext.x64.so");
            File.Copy(grpcSource, destinationPath, overwrite: true);
            Debug.Log($"Installed gRPC library to {destinationPath}");
        }
    }
}
