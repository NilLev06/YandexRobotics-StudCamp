using System.Collections.Generic;
using System.IO;
using Unity.InferenceEngine;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Prepares FixedArm scene for visual validation of a trained ONNX policy.
/// Menu: GFS-X → Visual Validation → ...
/// </summary>
public static class VisualValidationSetup
{
    private const string FixedScenePath = "Assets/Scenes/P3_DigitalTwin_FixedArm.unity";
    private const string RobotPrefabPath = "Assets/Prefabs/GFSX_Robot.prefab";
    private const string ValidationModelsDir = "Assets/Models/Validation";
    private const string RobotObjectName = "GFS-X Robot";
    private const string DefaultRunId = "fixed_arm_30m_relay_x4";
    private const string OnnxFileName = "GFSX_Brain_Fixed.onnx";

    [MenuItem("GFS-X/Visual Validation/Fixed Arm (fixed_arm_30m_relay_x4)")]
    [MenuItem("GFS-X/Validate Fixed Arm Now", false, 10)]
    public static void SetupFixedArm30mRelay()
    {
        Setup(FixedScenePath, DefaultRunId, OnnxFileName, showDialog: true);
    }

    [MenuItem("GFS-X/Visual Validation/Fixed Arm (fixed_arm_10m_relay_x4)")]
    public static void SetupFixedArm10mRelay()
    {
        Setup(FixedScenePath, "fixed_arm_10m_relay_x4", OnnxFileName, showDialog: true);
    }

    [MenuItem("GFS-X/Visual Validation/Fixed Arm (fixed_arm_5m_simple_x4)")]
    public static void SetupFixedArm5mSimple()
    {
        Setup(FixedScenePath, "fixed_arm_5m_simple_x4", OnnxFileName, showDialog: true);
    }

    [MenuItem("GFS-X/Visual Validation/Fixed Arm (fixed_arm_3m_x4)")]
    public static void SetupFixedArm3mX4()
    {
        Setup(FixedScenePath, "fixed_arm_3m_x4", OnnxFileName, showDialog: true);
    }

    [MenuItem("GFS-X/Visual Validation/Fixed Arm (fixed_arm_15m_x4)")]
    public static void SetupFixedArm15mX4()
    {
        Setup(FixedScenePath, "fixed_arm_15m_x4", OnnxFileName, showDialog: true);
    }

    [MenuItem("GFS-X/Visual Validation/Reset Fixed Arm To Training")]
    public static void ResetFixedToTraining()
    {
        ResetToTraining(FixedScenePath);
    }

    /// <summary>Unity batchmode: -executeMethod VisualValidationSetup.SetupBatch</summary>
    public static void SetupBatch()
    {
        string runId = DefaultRunId;
        string envRun = System.Environment.GetEnvironmentVariable("GFSX_VALIDATION_RUN_ID");
        if (!string.IsNullOrWhiteSpace(envRun))
            runId = envRun.Trim();

        bool ok = Setup(FixedScenePath, runId, OnnxFileName, showDialog: false);
        EditorApplication.Exit(ok ? 0 : 1);
    }

    private static bool Setup(string scenePath, string runId, string onnxFileName, bool showDialog)
    {
        string projectRoot = Directory.GetParent(Application.dataPath).FullName;
        string sourceOnnx = Path.Combine(projectRoot, "results", runId, onnxFileName);
        if (!File.Exists(sourceOnnx))
        {
            string msg = $"ONNX not found:\n{sourceOnnx}\n\nCheck run-id and that training finished.";
            Debug.LogError(msg);
            if (showDialog)
                EditorUtility.DisplayDialog("Visual Validation", msg, "OK");
            return false;
        }

        Directory.CreateDirectory(Path.Combine(projectRoot, ValidationModelsDir));
        string assetPath =
            $"{ValidationModelsDir}/{Path.GetFileNameWithoutExtension(onnxFileName)}_{runId}.onnx";
        File.Copy(sourceOnnx, Path.Combine(projectRoot, assetPath), true);
        AssetDatabase.ImportAsset(assetPath, ImportAssetOptions.ForceUpdate);

        ModelAsset model = AssetDatabase.LoadAssetAtPath<ModelAsset>(assetPath);
        if (model == null)
        {
            string msg = $"Failed to import ONNX as ModelAsset:\n{assetPath}";
            Debug.LogError(msg);
            if (showDialog)
                EditorUtility.DisplayDialog("Visual Validation", msg, "OK");
            return false;
        }

        // P3 uses a PrefabInstance — stamp the prefab first (reliable in Editor).
        bool prefabOk = ApplyInferenceToPrefabAsset(model);
        if (!prefabOk)
        {
            string msg = $"No RobotBrain on prefab:\n{RobotPrefabPath}";
            Debug.LogError(msg);
            if (showDialog)
                EditorUtility.DisplayDialog("Visual Validation", msg, "OK");
            return false;
        }

        Scene scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
        if (!scene.IsValid())
        {
            string msg = $"Failed to open scene:\n{scenePath}";
            Debug.LogError(msg);
            if (showDialog)
                EditorUtility.DisplayDialog("Visual Validation", msg, "OK");
            return false;
        }

        // Also update any live scene instances (optional; prefab already set).
        RobotBrain[] agents = FindRobotBrainsInOpenScenes();
        foreach (RobotBrain agent in agents)
            ApplyInferenceModel(agent, model);

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        AssetDatabase.SaveAssets();

        Debug.Log(
            $"Visual validation ready: scene={scenePath}, run={runId}, model={assetPath}, " +
            $"prefab={RobotPrefabPath}, sceneAgents={agents.Length}. Press Play.");

        if (showDialog)
        {
            EditorUtility.DisplayDialog(
                "Visual Validation Ready",
                "Configured for inference:\n" +
                "- Mode: FixedArm\n" +
                $"- Run: {runId}\n" +
                $"- Model: {assetPath}\n" +
                "- Prefab GFSX_Robot: Inference Only\n" +
                $"- Scene agents updated: {agents.Length}\n\n" +
                "Press Play in Unity to watch the agent.",
                "OK");
        }

        return true;
    }

    private static void ResetToTraining(string scenePath)
    {
        ResetPrefabToTraining();

        Scene scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
        if (!scene.IsValid())
            return;

        RobotBrain[] agents = FindRobotBrainsInOpenScenes();
        foreach (RobotBrain agent in agents)
            ApplyTrainingMode(agent);

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        AssetDatabase.SaveAssets();
        Debug.Log($"Reset {scenePath} + prefab to training mode (Default behavior).");
    }

    private static bool ApplyInferenceToPrefabAsset(ModelAsset model)
    {
        GameObject root = PrefabUtility.LoadPrefabContents(RobotPrefabPath);
        try
        {
            RobotBrain brain = root.GetComponent<RobotBrain>();
            if (brain == null)
                brain = root.GetComponentInChildren<RobotBrain>(true);
            if (brain == null)
                return false;

            ApplyInferenceModel(brain, model);
            PrefabUtility.SaveAsPrefabAsset(root, RobotPrefabPath);
            Debug.Log($"Prefabricated inference model onto {RobotPrefabPath}");
            return true;
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(root);
        }
    }

    private static void ResetPrefabToTraining()
    {
        GameObject root = PrefabUtility.LoadPrefabContents(RobotPrefabPath);
        try
        {
            RobotBrain brain = root.GetComponent<RobotBrain>();
            if (brain == null)
                brain = root.GetComponentInChildren<RobotBrain>(true);
            if (brain == null)
                return;

            ApplyTrainingMode(brain);
            PrefabUtility.SaveAsPrefabAsset(root, RobotPrefabPath);
        }
        finally
        {
            PrefabUtility.UnloadPrefabContents(root);
        }
    }

    private static void ApplyInferenceModel(RobotBrain agent, ModelAsset model)
    {
        SerializedObject brainObject = new SerializedObject(agent);
        SerializedProperty mode = brainObject.FindProperty("trainingMode");
        if (mode != null)
            mode.enumValueIndex = (int)RobotTrainingMode.FixedArm;
        SerializedProperty dr = brainObject.FindProperty("enableDomainRandomization");
        if (dr != null)
            dr.boolValue = false;
        brainObject.ApplyModifiedPropertiesWithoutUndo();

        BehaviorParameters behavior = agent.GetComponent<BehaviorParameters>();
        if (behavior == null)
        {
            Debug.LogWarning($"RobotBrain on {agent.name} has no BehaviorParameters — skipped.");
            return;
        }

        behavior.BehaviorName = "GFSX_Brain_Fixed";
        behavior.BehaviorType = BehaviorType.InferenceOnly;
        behavior.Model = model;
        behavior.DeterministicInference = true;
        EditorUtility.SetDirty(behavior);
        EditorUtility.SetDirty(agent);
    }

    private static void ApplyTrainingMode(RobotBrain agent)
    {
        SerializedObject brainObject = new SerializedObject(agent);
        SerializedProperty mode = brainObject.FindProperty("trainingMode");
        if (mode != null)
            mode.enumValueIndex = (int)RobotTrainingMode.FixedArm;
        SerializedProperty dr = brainObject.FindProperty("enableDomainRandomization");
        if (dr != null)
            dr.boolValue = true;
        brainObject.ApplyModifiedPropertiesWithoutUndo();

        BehaviorParameters behavior = agent.GetComponent<BehaviorParameters>();
        if (behavior == null)
            return;

        behavior.BehaviorType = BehaviorType.Default;
        behavior.Model = null;
        EditorUtility.SetDirty(behavior);
        EditorUtility.SetDirty(agent);
    }

    private static RobotBrain[] FindRobotBrainsInOpenScenes()
    {
        var list = new List<RobotBrain>();

        RobotBrain[] found = Object.FindObjectsByType<RobotBrain>(
            FindObjectsInactive.Include,
            FindObjectsSortMode.None);
        if (found != null)
        {
            for (int i = 0; i < found.Length; i++)
            {
                if (found[i] != null && found[i].gameObject.scene.IsValid())
                    list.Add(found[i]);
            }
        }

        if (list.Count == 0)
        {
            GameObject robot = GameObject.Find(RobotObjectName);
            if (robot != null)
            {
                RobotBrain brain = robot.GetComponent<RobotBrain>();
                if (brain == null)
                    brain = robot.GetComponentInChildren<RobotBrain>(true);
                if (brain != null)
                    list.Add(brain);
            }
        }

        if (list.Count == 0)
        {
            RobotBrain[] all = Resources.FindObjectsOfTypeAll<RobotBrain>();
            for (int i = 0; i < all.Length; i++)
            {
                RobotBrain brain = all[i];
                if (brain == null)
                    continue;
                if (!brain.gameObject.scene.IsValid())
                    continue;
                if (brain.gameObject.scene.path != FixedScenePath &&
                    brain.gameObject.scene.path != SceneManager.GetActiveScene().path)
                {
                    continue;
                }

                list.Add(brain);
            }
        }

        return list.ToArray();
    }
}
