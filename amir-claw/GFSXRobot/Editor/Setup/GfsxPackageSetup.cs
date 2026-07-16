using System;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Adds only the tag required by the optional claw-grasp workflow. The full
/// TagManager asset is deliberately not shipped because importing somebody
/// else's ProjectSettings would overwrite unrelated project configuration.
/// </summary>
[InitializeOnLoad]
public static class GfsxPackageSetup
{
    public const string TargetBallTag = "TargetBall";

    static GfsxPackageSetup()
    {
        EditorApplication.delayCall += EnsureTargetBallTag;
    }

    [MenuItem("Tools/GFS-X/Ensure TargetBall Tag")]
    public static void EnsureTargetBallTag()
    {
        try
        {
            UnityEngine.Object[] assets = AssetDatabase.LoadAllAssetsAtPath(
                "ProjectSettings/TagManager.asset");
            if (assets == null || assets.Length == 0)
                return;

            SerializedObject tagManager = new SerializedObject(assets[0]);
            SerializedProperty tags = tagManager.FindProperty("tags");
            if (tags == null)
                return;

            for (int index = 0; index < tags.arraySize; index++)
            {
                SerializedProperty existing =
                    tags.GetArrayElementAtIndex(index);
                if (string.Equals(
                        existing.stringValue,
                        TargetBallTag,
                        StringComparison.Ordinal))
                {
                    return;
                }
            }

            int newIndex = tags.arraySize;
            tags.InsertArrayElementAtIndex(newIndex);
            tags.GetArrayElementAtIndex(newIndex).stringValue = TargetBallTag;
            tagManager.ApplyModifiedProperties();
            AssetDatabase.SaveAssets();

            Debug.Log(
                "GFS-X package setup: added the TargetBall tag used by " +
                "the optional claw-grasp workflow.");
        }
        catch (Exception exception)
        {
            Debug.LogWarning(
                "GFS-X package setup could not add TargetBall automatically. " +
                "Add it manually in Edit > Project Settings > Tags and Layers. " +
                exception.Message);
        }
    }
}
