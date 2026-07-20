using UnityEditor;
using UnityEditor.SceneManagement;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Configures two training scenes:
/// P2 — mobile arm (6 continuous actions), P3 — fixed arm (3 continuous actions).
/// Menu: GFS-X → Setup Training Scenes
/// Batch: Unity -batchmode -quit -executeMethod SetupTrainingScenes.SetupBatch
/// </summary>
public static class SetupTrainingScenes
{
    private const string MobileScenePath = "Assets/Scenes/P2_DigitalTwin.unity";
    private const string FixedScenePath = "Assets/Scenes/P3_DigitalTwin_FixedArm.unity";
    private const string MobileBehaviorName = "GFSX_Brain_Mobile";
    private const string FixedBehaviorName = "GFSX_Brain_Fixed";

    [MenuItem("GFS-X/Setup Training Scenes")]
    public static void SetupFromMenu()
    {
        if (!ConfigureAllScenes())
            return;

        EditorSceneManager.SaveOpenScenes();
        Debug.Log("Training scenes configured and saved.");
    }

    public static void SetupBatch()
    {
        if (!ConfigureAllScenes())
        {
            EditorApplication.Exit(1);
            return;
        }

        Debug.Log("SetupTrainingScenes.SetupBatch succeeded.");
        EditorApplication.Exit(0);
    }

    private static bool ConfigureAllScenes()
    {
        if (!ConfigureScene(MobileScenePath, RobotTrainingMode.MobileArm, MobileBehaviorName, 6))
            return false;

        if (!System.IO.File.Exists(FixedScenePath))
        {
            if (!AssetDatabase.CopyAsset(MobileScenePath, FixedScenePath))
            {
                Debug.LogError($"Failed to copy {MobileScenePath} to {FixedScenePath}.");
                return false;
            }

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
        }

        if (!ConfigureScene(FixedScenePath, RobotTrainingMode.FixedArm, FixedBehaviorName, 3))
            return false;

        AddSceneToBuildSettings(MobileScenePath);
        AddSceneToBuildSettings(FixedScenePath);
        AssetDatabase.SaveAssets();
        return true;
    }

    private static bool ConfigureScene(
        string scenePath,
        RobotTrainingMode mode,
        string behaviorName,
        int continuousActions)
    {
        Scene scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
        if (!scene.IsValid())
        {
            Debug.LogError($"Failed to open scene: {scenePath}");
            return false;
        }

        RobotBrain[] agents = Object.FindObjectsByType<RobotBrain>(FindObjectsSortMode.None);
        if (agents.Length == 0)
        {
            Debug.LogError($"No RobotBrain found in {scenePath}");
            return false;
        }

        foreach (RobotBrain agent in agents)
        {
            ConfigureAgent(agent, mode, behaviorName, continuousActions);
        }

        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            Debug.LogError($"Failed to save scene: {scenePath}");
            return false;
        }

        Debug.Log($"Configured {scenePath}: mode={mode}, behavior={behaviorName}, continuous={continuousActions}");
        return true;
    }

    private static void ConfigureAgent(
        RobotBrain agent,
        RobotTrainingMode mode,
        string behaviorName,
        int continuousActions)
    {
        SerializedObject brainObject = new SerializedObject(agent);
        brainObject.FindProperty("trainingMode").enumValueIndex = (int)mode;
        brainObject.ApplyModifiedPropertiesWithoutUndo();

        BehaviorParameters behaviorParameters = agent.GetComponent<BehaviorParameters>();
        if (behaviorParameters == null)
        {
            Debug.LogWarning($"RobotBrain on {agent.name} has no BehaviorParameters.");
            return;
        }

        behaviorParameters.BehaviorName = behaviorName;
        behaviorParameters.BrainParameters.ActionSpec =
            new ActionSpec(continuousActions, new[] { 3 });
        behaviorParameters.BrainParameters.VectorObservationSize = 15;
        behaviorParameters.BrainParameters.NumStackedVectorObservations = 4;

        EditorUtility.SetDirty(behaviorParameters);
        EditorUtility.SetDirty(agent);

        VirtualSensors sensors = agent.GetComponent<VirtualSensors>();
        if (sensors != null)
        {
            SerializedObject sensorsObject = new SerializedObject(sensors);
            GfsxArmRigController armRig = agent.GetComponentInChildren<GfsxArmRigController>(true);
            if (armRig != null)
            {
                sensorsObject.FindProperty("armOcclusionRoot").objectReferenceValue = armRig.transform;
                sensorsObject.ApplyModifiedPropertiesWithoutUndo();
                EditorUtility.SetDirty(sensors);
            }
        }
    }

    private static void AddSceneToBuildSettings(string scenePath)
    {
        EditorBuildSettingsScene[] existing = EditorBuildSettings.scenes;
        for (int index = 0; index < existing.Length; index++)
        {
            if (existing[index].path == scenePath)
                return;
        }

        EditorBuildSettingsScene[] updated = new EditorBuildSettingsScene[existing.Length + 1];
        for (int index = 0; index < existing.Length; index++)
            updated[index] = existing[index];

        updated[existing.Length] = new EditorBuildSettingsScene(scenePath, true);
        EditorBuildSettings.scenes = updated;
    }
}
