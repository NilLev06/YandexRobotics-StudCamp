using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Creates / refreshes the FixedArm robot prefab from P3 scene.
/// Menu: GFS-X → Create / Update GFS-X Robot Prefab
/// </summary>
public static class CreateGfsxRobotPrefab
{
    private const string FixedScenePath = "Assets/Scenes/P3_DigitalTwin_FixedArm.unity";
    private const string PrefabDir = "Assets/Prefabs";
    private const string PrefabPath = "Assets/Prefabs/GFSX_Robot.prefab";
    private const string RobotObjectName = "GFS-X Robot";

    [MenuItem("GFS-X/Create / Update GFS-X Robot Prefab")]
    public static void CreateFromMenu()
    {
        int code = CreateOrUpdate(replaceSceneInstance: true);
        if (code != 0)
            EditorUtility.DisplayDialog("GFS-X Prefab", "Failed — see Console.", "OK");
        else
            EditorUtility.DisplayDialog(
                "GFS-X Prefab",
                $"Prefab ready:\n{PrefabPath}\n\nP3 scene instance linked to prefab.",
                "OK");
    }

    /// <summary>Unity batchmode: -executeMethod CreateGfsxRobotPrefab.CreateBatch</summary>
    public static void CreateBatch()
    {
        EditorApplication.Exit(CreateOrUpdate(replaceSceneInstance: true));
    }

    public static int CreateOrUpdate(bool replaceSceneInstance)
    {
        Directory.CreateDirectory(
            Path.Combine(Directory.GetParent(Application.dataPath).FullName, "Assets", "Prefabs"));

        Scene scene = EditorSceneManager.OpenScene(FixedScenePath, OpenSceneMode.Single);
        if (!scene.IsValid())
        {
            Debug.LogError($"CreateGfsxRobotPrefab: cannot open {FixedScenePath}");
            return 1;
        }

        GameObject robot = GameObject.Find(RobotObjectName);
        if (robot == null)
        {
            Debug.LogError($"CreateGfsxRobotPrefab: '{RobotObjectName}' not found in scene.");
            return 2;
        }

        RobotBrain brain = robot.GetComponent<RobotBrain>();
        if (brain == null)
        {
            Debug.LogError("CreateGfsxRobotPrefab: RobotBrain missing on robot.");
            return 3;
        }

        // Ensure FixedArm defaults on the source before baking the prefab.
        SerializedObject brainObject = new SerializedObject(brain);
        SerializedProperty mode = brainObject.FindProperty("trainingMode");
        if (mode != null)
            mode.enumValueIndex = (int)RobotTrainingMode.FixedArm;
        brainObject.ApplyModifiedPropertiesWithoutUndo();

        GfsxArmRigController arm = robot.GetComponent<GfsxArmRigController>();
        if (arm != null)
            arm.SetFloorPickupPose();

        PrefabUtility.SaveAsPrefabAssetAndConnect(
            robot,
            PrefabPath,
            InteractionMode.AutomatedAction,
            out bool success);

        if (!success)
        {
            // Fallback without reconnect if connect fails on nested setups.
            GameObject saved = PrefabUtility.SaveAsPrefabAsset(robot, PrefabPath, out success);
            if (!success || saved == null)
            {
                Debug.LogError($"CreateGfsxRobotPrefab: failed to write {PrefabPath}");
                return 4;
            }

            if (replaceSceneInstance)
            {
                Vector3 pos = robot.transform.position;
                Quaternion rot = robot.transform.rotation;
                Transform parent = robot.transform.parent;
                Object.DestroyImmediate(robot);
                GameObject instance = (GameObject)PrefabUtility.InstantiatePrefab(
                    AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath),
                    scene);
                instance.name = RobotObjectName;
                instance.transform.SetParent(parent, true);
                instance.transform.SetPositionAndRotation(pos, rot);
            }
        }

        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);

        Debug.Log($"CreateGfsxRobotPrefab: wrote {PrefabPath} and updated {FixedScenePath}");
        return 0;
    }
}
