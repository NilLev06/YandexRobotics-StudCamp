using UnityEditor;
using UnityEditor.SceneManagement;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Configures FixedArm training scene P3 (3 continuous: drive + camera pan).
/// Menu: GFS-X → Setup Training Scenes
/// </summary>
public static class SetupTrainingScenes
{
    private const string FixedScenePath = "Assets/Scenes/P3_DigitalTwin_FixedArm.unity";
    private const string FixedBehaviorName = "GFSX_Brain_Fixed";
    private const int ContinuousActions = 4;
    private const int VectorObservations = 16;

    [MenuItem("GFS-X/Setup Training Scenes")]
    public static void SetupFromMenu()
    {
        if (!ConfigureFixedScene())
            return;

        EditorSceneManager.SaveOpenScenes();
        Debug.Log("FixedArm training scene configured and saved.");
    }

    public static void SetupBatch()
    {
        if (!ConfigureFixedScene())
        {
            EditorApplication.Exit(1);
            return;
        }

        Debug.Log("SetupTrainingScenes.SetupBatch succeeded.");
        EditorApplication.Exit(0);
    }

    private static bool ConfigureFixedScene()
    {
        if (!ConfigureScene(FixedScenePath, FixedBehaviorName, ContinuousActions))
            return false;

        AddSceneToBuildSettings(FixedScenePath);
        AssetDatabase.SaveAssets();
        return true;
    }

    private static bool ConfigureScene(
        string scenePath,
        string behaviorName,
        int continuousActions)
    {
        Scene scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
        if (!scene.IsValid())
        {
            Debug.LogError($"Failed to open scene: {scenePath}");
            return false;
        }

        RobotBrain[] agents = Object.FindObjectsByType<RobotBrain>(
            FindObjectsInactive.Include,
            FindObjectsSortMode.None);
        if (agents.Length == 0)
        {
            Debug.LogError($"No RobotBrain found in {scenePath}");
            return false;
        }

        foreach (RobotBrain agent in agents)
            ConfigureAgent(agent, behaviorName, continuousActions);

        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            Debug.LogError($"Failed to save scene: {scenePath}");
            return false;
        }

        Debug.Log(
            $"Configured {scenePath}: FixedArm, behavior={behaviorName}, " +
            $"continuous={continuousActions}, vectorObs={VectorObservations}");
        return true;
    }

    private static void ConfigureAgent(
        RobotBrain agent,
        string behaviorName,
        int continuousActions)
    {
        SerializedObject brainObject = new SerializedObject(agent);
        brainObject.FindProperty("trainingMode").enumValueIndex = (int)RobotTrainingMode.FixedArm;
        brainObject.ApplyModifiedPropertiesWithoutUndo();

        BehaviorParameters behaviorParameters = agent.GetComponent<BehaviorParameters>();
        if (behaviorParameters == null)
        {
            Debug.LogWarning($"RobotBrain on {agent.name} has no BehaviorParameters.");
            return;
        }

        behaviorParameters.BehaviorName = behaviorName;
        behaviorParameters.BrainParameters.ActionSpec = new ActionSpec(
            continuousActions,
            new[] { 3 });
        behaviorParameters.BrainParameters.VectorObservationSize = VectorObservations;
        EditorUtility.SetDirty(behaviorParameters);
        EditorUtility.SetDirty(agent);
    }

    private static void AddSceneToBuildSettings(string scenePath)
    {
        EditorBuildSettingsScene[] scenes = EditorBuildSettings.scenes;
        for (int i = 0; i < scenes.Length; i++)
        {
            if (scenes[i].path == scenePath)
            {
                scenes[i].enabled = true;
                EditorBuildSettings.scenes = scenes;
                return;
            }
        }

        var list = new System.Collections.Generic.List<EditorBuildSettingsScene>(scenes)
        {
            new EditorBuildSettingsScene(scenePath, true)
        };
        EditorBuildSettings.scenes = list.ToArray();
    }
}
