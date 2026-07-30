using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

[CustomEditor(typeof(GfsxArmRigController))]
public sealed class GfsxArmRigControllerEditor : Editor
{
    private SerializedProperty s1Angle;
    private SerializedProperty s2Angle;
    private SerializedProperty s3WristRollAngle;
    private SerializedProperty s4Closure;
    private SerializedProperty s1MinimumAngle;
    private SerializedProperty s1MaximumAngle;
    private SerializedProperty s2MinimumAngle;
    private SerializedProperty s2MaximumAngle;
    private SerializedProperty s3MinimumAngle;
    private SerializedProperty s3MaximumAngle;
    private SerializedProperty s4MinimumClosureCommand;
    private SerializedProperty s4MaximumClosureLimit;
    private SerializedProperty closedJawOpeningAngle;
    private SerializedProperty openJawOpeningAngle;
    private bool showAdvancedSetup;

    private void OnEnable()
    {
        s1Angle = serializedObject.FindProperty("s1Angle");
        s2Angle = serializedObject.FindProperty("s2Angle");
        s3WristRollAngle = serializedObject.FindProperty(
            "s3WristRollAngle");
        s4Closure = serializedObject.FindProperty("s4Closure");
        s1MinimumAngle = serializedObject.FindProperty("s1MinimumAngle");
        s1MaximumAngle = serializedObject.FindProperty("s1MaximumAngle");
        s2MinimumAngle = serializedObject.FindProperty("s2MinimumAngle");
        s2MaximumAngle = serializedObject.FindProperty("s2MaximumAngle");
        s3MinimumAngle = serializedObject.FindProperty("s3MinimumAngle");
        s3MaximumAngle = serializedObject.FindProperty("s3MaximumAngle");
        s4MinimumClosureCommand = serializedObject.FindProperty(
            "s4MinimumClosureCommand");
        s4MaximumClosureLimit = serializedObject.FindProperty(
            "s4MaximumClosureLimit");
        closedJawOpeningAngle = serializedObject.FindProperty(
            "closedJawOpeningAngle");
        openJawOpeningAngle = serializedObject.FindProperty(
            "openJawOpeningAngle");
    }

    public override void OnInspectorGUI()
    {
        GfsxArmRigController controller = (GfsxArmRigController)target;
        serializedObject.Update();

        EditorGUILayout.HelpBox(
            "REAL ARM CHAIN: S1 shoulder, S2 elbow bend, S3 axial wrist " +
            "roll, and S4 jaws. The old fake S2 pivot remains removed; the " +
            "existing bend is now labelled correctly and the missing wrist " +
            "shaft is independent. All non-servo transforms remain locked.",
            MessageType.Info);

        EditorGUI.BeginChangeCheck();

        EditorGUILayout.LabelField(
            "Allowed servo limits",
            EditorStyles.boldLabel);
        EditorGUILayout.HelpBox(
            "Edit Min and Max here. These limits are used by the Inspector, " +
            "the in-game HUD, keyboard control, and external commands. " +
            "S1-S3 are angle offsets from neutral. For S4, Min is fully " +
            "open and Max is fully closed.",
            MessageType.None);
        DrawAllowedAngleRange(
            "S1 Shoulder",
            s1MinimumAngle,
            s1MaximumAngle);
        DrawAllowedAngleRange(
            "S2 Elbow bend",
            s2MinimumAngle,
            s2MaximumAngle);
        DrawAllowedAngleRange(
            "S3 Wrist roll",
            s3MinimumAngle,
            s3MaximumAngle);
        DrawPositiveRange(
            "S4 Jaw closure",
            s4MinimumClosureCommand,
            s4MaximumClosureLimit);

        EditorGUILayout.Space(7f);
        EditorGUILayout.LabelField(
            "Current servo positions",
            EditorStyles.boldLabel);
        s1Angle.floatValue = EditorGUILayout.Slider(
            "S1 Shoulder (degrees)",
            s1Angle.floatValue,
            s1MinimumAngle.floatValue,
            s1MaximumAngle.floatValue);
        s2Angle.floatValue = EditorGUILayout.Slider(
            "S2 Elbow bend (degrees)",
            s2Angle.floatValue,
            s2MinimumAngle.floatValue,
            s2MaximumAngle.floatValue);
        s3WristRollAngle.floatValue = EditorGUILayout.Slider(
            "S3 Wrist roll (degrees)",
            s3WristRollAngle.floatValue,
            s3MinimumAngle.floatValue,
            s3MaximumAngle.floatValue);
        s4Closure.floatValue = EditorGUILayout.Slider(
            "S4 Jaw closure",
            s4Closure.floatValue,
            s4MinimumClosureCommand.floatValue,
            s4MaximumClosureLimit.floatValue);

        EditorGUILayout.Space(5f);
        EditorGUILayout.LabelField(
            "S4 physical travel",
            EditorStyles.boldLabel);
        closedJawOpeningAngle.floatValue = EditorGUILayout.Slider(
            "Closed jaw angle",
            closedJawOpeningAngle.floatValue,
            0f,
            15f);
        openJawOpeningAngle.floatValue = EditorGUILayout.Slider(
            "Open jaw angle",
            openJawOpeningAngle.floatValue,
            20f,
            45f);
        bool servoChanged = EditorGUI.EndChangeCheck();

        serializedObject.ApplyModifiedProperties();

        if (servoChanged)
            CommitPose(controller);

        EditorGUILayout.Space(6f);
        using (new EditorGUILayout.HorizontalScope())
        {
            if (GUILayout.Button("Reset servos"))
            {
                Undo.RecordObject(controller, "Reset GFS-X servos");
                controller.ResetToNeutral();
                CommitPose(controller);
            }

            if (GUILayout.Button("Open jaws"))
            {
                Undo.RecordObject(controller, "Open GFS-X jaws");
                controller.SetServoCommands(
                    controller.S1Angle,
                    controller.S2Angle,
                    controller.S3Angle,
                    controller.S4MinimumClosureLimit);
                CommitPose(controller);
            }

            if (GUILayout.Button("Close jaws"))
            {
                Undo.RecordObject(controller, "Close GFS-X jaws");
                controller.SetServoCommands(
                    controller.S1Angle,
                    controller.S2Angle,
                    controller.S3Angle,
                    controller.S4MaximumClosureLimit);
                CommitPose(controller);
            }
        }

        EditorGUILayout.Space(8f);
        showAdvancedSetup = EditorGUILayout.Foldout(
            showAdvancedSetup,
            "Advanced rig setup",
            true);

        if (!showAdvancedSetup)
            return;

        serializedObject.Update();
        DrawPropertiesExcluding(
            serializedObject,
            "m_Script",
            "s1Angle",
            "s2Angle",
            "s3WristRollAngle",
            "s4Closure",
            "s1MinimumAngle",
            "s1MaximumAngle",
            "s2MinimumAngle",
            "s2MaximumAngle",
            "s3MinimumAngle",
            "s3MaximumAngle",
            "s4MinimumClosureCommand",
            "s4MaximumClosureLimit",
            "closedJawOpeningAngle",
            "openJawOpeningAngle");

        if (serializedObject.ApplyModifiedProperties())
            CommitPose(controller);
    }

    private static void DrawAllowedAngleRange(
        string label,
        SerializedProperty minimum,
        SerializedProperty maximum)
    {
        using (new EditorGUILayout.HorizontalScope())
        {
            EditorGUILayout.LabelField(label, GUILayout.Width(125f));
            EditorGUILayout.LabelField("Min", GUILayout.Width(25f));
            minimum.floatValue = EditorGUILayout.FloatField(
                minimum.floatValue,
                GUILayout.MinWidth(45f));
            EditorGUILayout.LabelField("Max", GUILayout.Width(29f));
            maximum.floatValue = EditorGUILayout.FloatField(
                maximum.floatValue,
                GUILayout.MinWidth(45f));
        }

        minimum.floatValue = Mathf.Clamp(minimum.floatValue, -180f, 0f);
        maximum.floatValue = Mathf.Clamp(maximum.floatValue, 0f, 180f);
        if (maximum.floatValue - minimum.floatValue < 1f)
            maximum.floatValue = Mathf.Min(180f, minimum.floatValue + 1f);
    }

    private static void DrawPositiveRange(
        string label,
        SerializedProperty minimum,
        SerializedProperty maximum)
    {
        using (new EditorGUILayout.HorizontalScope())
        {
            EditorGUILayout.LabelField(label, GUILayout.Width(125f));
            EditorGUILayout.LabelField("Min", GUILayout.Width(25f));
            minimum.floatValue = EditorGUILayout.FloatField(
                minimum.floatValue,
                GUILayout.MinWidth(45f));
            EditorGUILayout.LabelField("Max", GUILayout.Width(29f));
            maximum.floatValue = EditorGUILayout.FloatField(
                maximum.floatValue,
                GUILayout.MinWidth(45f));
        }

        minimum.floatValue = Mathf.Clamp(minimum.floatValue, 0f, 179f);
        maximum.floatValue = Mathf.Clamp(maximum.floatValue, 1f, 180f);
        if (maximum.floatValue - minimum.floatValue < 1f)
            maximum.floatValue = Mathf.Min(180f, minimum.floatValue + 1f);
    }

    private static void CommitPose(GfsxArmRigController controller)
    {
        controller.ApplyPose();
        EditorUtility.SetDirty(controller);

        if (!Application.isPlaying && controller.gameObject.scene.IsValid())
            EditorSceneManager.MarkSceneDirty(controller.gameObject.scene);

        SceneView.RepaintAll();
    }
}
