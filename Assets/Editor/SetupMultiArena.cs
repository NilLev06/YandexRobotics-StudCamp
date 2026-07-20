using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Groups Ground / walls / TargetBall / robot under Arena_0 and adds MultiArenaManager.
/// Menu: GFS-X → Setup Multi-Arena Scene
/// </summary>
public static class SetupMultiArena
{
    private const string ScenePath = "Assets/Scenes/P2_DigitalTwin.unity";

    [MenuItem("GFS-X/Setup Multi-Arena Scene")]
    public static void SetupFromMenu()
    {
        if (!SetupScene())
            return;

        EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());
        EditorSceneManager.SaveOpenScenes();
        Debug.Log("Multi-arena scene setup complete and saved.");
    }

    /// <summary>
    /// Batchmode entry: Unity -batchmode -quit -executeMethod SetupMultiArena.SetupBatch
    /// </summary>
    public static void SetupBatch()
    {
        var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        if (!scene.IsValid())
        {
            Debug.LogError($"Failed to open {ScenePath}");
            EditorApplication.Exit(1);
            return;
        }

        if (!SetupScene())
        {
            EditorApplication.Exit(1);
            return;
        }

        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            Debug.LogError("Failed to save scene.");
            EditorApplication.Exit(1);
            return;
        }

        Debug.Log("SetupMultiArena.SetupBatch succeeded.");
        EditorApplication.Exit(0);
    }

    private static bool SetupScene()
    {
        TrainingArena existing = Object.FindFirstObjectByType<TrainingArena>();
        if (existing != null)
        {
            EnsureManager(existing);
            WireArena(existing);
            Debug.Log("TrainingArena already present — rewired references.");
            return true;
        }

        GameObject robot = GameObject.Find("GFS-X Robot");
        GameObject ground = GameObject.Find("Ground");
        GameObject ball = GameObject.Find("TargetBall");
        GameObject wall0 = GameObject.Find("Cube");
        GameObject wall1 = GameObject.Find("Cube (1)");
        GameObject wall2 = GameObject.Find("Cube (2)");
        GameObject wall3 = GameObject.Find("Cube (3)");

        if (robot == null || ground == null || ball == null)
        {
            Debug.LogError("SetupMultiArena: missing GFS-X Robot / Ground / TargetBall.");
            return false;
        }

        GameObject arenaObject = new GameObject("Arena_0");
        Undo.RegisterCreatedObjectUndo(arenaObject, "Create Arena_0");

        Parent(ground, arenaObject);
        Parent(ball, arenaObject);
        Parent(robot, arenaObject);
        if (wall0 != null) Parent(wall0, arenaObject);
        if (wall1 != null) Parent(wall1, arenaObject);
        if (wall2 != null) Parent(wall2, arenaObject);
        if (wall3 != null) Parent(wall3, arenaObject);

        TrainingArena arena = arenaObject.AddComponent<TrainingArena>();
        WireArena(arena);
        EnsureManager(arena);
        return true;
    }

    private static void Parent(GameObject child, GameObject parent)
    {
        Undo.SetTransformParent(child.transform, parent.transform, "Parent under Arena");
    }

    private static void WireArena(TrainingArena arena)
    {
        RobotBrain brain = arena.GetComponentInChildren<RobotBrain>(true);
        Transform ball = null;
        foreach (Transform t in arena.GetComponentsInChildren<Transform>(true))
        {
            if (t.CompareTag("TargetBall"))
            {
                ball = t;
                break;
            }
        }

        SimulatedYoloCamera camera = arena.GetComponentInChildren<SimulatedYoloCamera>(true);
        SerializedObject so = new SerializedObject(arena);
        so.FindProperty("agent").objectReferenceValue = brain;
        so.FindProperty("targetBall").objectReferenceValue = ball;
        so.FindProperty("ballBody").objectReferenceValue =
            ball != null ? ball.GetComponent<Rigidbody>() : null;
        so.FindProperty("yoloCamera").objectReferenceValue = camera;
        so.FindProperty("halfExtents").vector2Value = new Vector2(2.8f, 2.0f);
        so.FindProperty("spawnObstacles").boolValue = true;
        so.FindProperty("minObstacles").intValue = 6;
        so.FindProperty("maxObstacles").intValue = 12;
        so.FindProperty("obstacleSize").vector3Value = new Vector3(0.36f, 0.14f, 0.24f);
        so.FindProperty("randomizeRobotPose").boolValue = true;
        so.FindProperty("robotPassageWidth").floatValue = 0.55f;
        so.FindProperty("corridorHalfWidth").floatValue = 0.38f;
        so.FindProperty("ballScale").vector3Value = new Vector3(0.05f, 0.05f, 0.05f);
        so.FindProperty("ballColor").colorValue = new Color(1f, 0.45f, 0.05f, 1f);
        so.ApplyModifiedPropertiesWithoutUndo();

        if (brain != null)
        {
            SerializedObject brainSo = new SerializedObject(brain);
            SerializedProperty arenaProp = brainSo.FindProperty("trainingArena");
            if (arenaProp != null)
            {
                arenaProp.objectReferenceValue = arena;
                brainSo.ApplyModifiedPropertiesWithoutUndo();
            }
        }

        if (camera != null && ball != null)
        {
            SerializedObject camSo = new SerializedObject(camera);
            camSo.FindProperty("targetBall").objectReferenceValue = ball;
            camSo.ApplyModifiedPropertiesWithoutUndo();
        }
    }

    private static void EnsureManager(TrainingArena template)
    {
        MultiArenaManager manager = Object.FindFirstObjectByType<MultiArenaManager>();
        if (manager == null)
        {
            GameObject managerObject = new GameObject("MultiArenaManager");
            Undo.RegisterCreatedObjectUndo(managerObject, "Create MultiArenaManager");
            manager = managerObject.AddComponent<MultiArenaManager>();
        }

        SerializedObject so = new SerializedObject(manager);
        so.FindProperty("templateArena").objectReferenceValue = template;
        so.FindProperty("arenaCount").intValue = 40;
        so.FindProperty("columns").intValue = 8;
        so.FindProperty("spacing").vector2Value = new Vector2(9f, 7f);
        so.FindProperty("cloneOnAwake").boolValue = true;
        so.ApplyModifiedPropertiesWithoutUndo();
    }
}
