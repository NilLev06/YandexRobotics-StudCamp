using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

[CustomEditor(typeof(GfsxSensorHeadRigController))]
internal sealed class GfsxSensorHeadRigControllerEditor : Editor
{
    private SerializedProperty s5SensorYawPivot;
    private SerializedProperty s6CameraPitchPivot;
    private SerializedProperty ultrasoundVisual;
    private SerializedProperty cameraBaseVisual;
    private SerializedProperty cameraVisual;
    private SerializedProperty ultrasonicBeamPoint;

    private SerializedProperty s5PanAngle;
    private SerializedProperty s6TiltAngle;
    private SerializedProperty s5MinimumAngle;
    private SerializedProperty s5MaximumAngle;
    private SerializedProperty s6MinimumAngle;
    private SerializedProperty s6MaximumAngle;

    private SerializedProperty readKeyboard;
    private SerializedProperty degreesPerSecond;
    private SerializedProperty s5YawLocalAxis;
    private SerializedProperty s6PitchLocalAxis;

    private void OnEnable()
    {
        s5SensorYawPivot = serializedObject.FindProperty(
            "s5SensorYawPivot");
        s6CameraPitchPivot = serializedObject.FindProperty(
            "s6CameraPitchPivot");
        ultrasoundVisual = serializedObject.FindProperty(
            "ultrasoundVisual");
        cameraBaseVisual = serializedObject.FindProperty(
            "cameraBaseVisual");
        cameraVisual = serializedObject.FindProperty("cameraVisual");
        ultrasonicBeamPoint = serializedObject.FindProperty(
            "ultrasonicBeamPoint");

        s5PanAngle = serializedObject.FindProperty("s5PanAngle");
        s6TiltAngle = serializedObject.FindProperty("s6TiltAngle");
        s5MinimumAngle = serializedObject.FindProperty(
            "s5MinimumAngle");
        s5MaximumAngle = serializedObject.FindProperty(
            "s5MaximumAngle");
        s6MinimumAngle = serializedObject.FindProperty(
            "s6MinimumAngle");
        s6MaximumAngle = serializedObject.FindProperty(
            "s6MaximumAngle");

        readKeyboard = serializedObject.FindProperty("readKeyboard");
        degreesPerSecond = serializedObject.FindProperty(
            "degreesPerSecond");
        s5YawLocalAxis = serializedObject.FindProperty(
            "s5YawLocalAxis");
        s6PitchLocalAxis = serializedObject.FindProperty(
            "s6PitchLocalAxis");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        EditorGUILayout.PropertyField(s5SensorYawPivot);
        EditorGUILayout.PropertyField(s6CameraPitchPivot);
        EditorGUILayout.PropertyField(ultrasoundVisual);
        EditorGUILayout.PropertyField(cameraBaseVisual);
        EditorGUILayout.PropertyField(cameraVisual);
        EditorGUILayout.PropertyField(ultrasonicBeamPoint);

        EditorGUILayout.Space(7f);
        DrawHeader("Servo command sliders");
        DrawDynamicSlider(
            s5PanAngle,
            s5MinimumAngle,
            s5MaximumAngle,
            new GUIContent("S5 Pan Angle", "Keyboard: J / L"));
        DrawDynamicSlider(
            s6TiltAngle,
            s6MinimumAngle,
            s6MaximumAngle,
            new GUIContent("S6 Tilt Angle", "Keyboard: K / I"));

        bool centerRequested = GUILayout.Button("CENTER S5 / S6");

        EditorGUILayout.Space(7f);
        EditorGUILayout.PropertyField(s5MinimumAngle);
        EditorGUILayout.PropertyField(s5MaximumAngle);
        EditorGUILayout.PropertyField(s6MinimumAngle);
        EditorGUILayout.PropertyField(s6MaximumAngle);
        EditorGUILayout.HelpBox(
            "The manual identifies these as S5 and S6 but does not give " +
            "numerical limits or hardware-neutral angles. These values are " +
            "editable Unity calibration settings.",
            MessageType.Info);

        EditorGUILayout.Space(7f);
        EditorGUILayout.PropertyField(readKeyboard);
        EditorGUILayout.PropertyField(degreesPerSecond);

        EditorGUILayout.Space(7f);
        EditorGUILayout.PropertyField(s5YawLocalAxis);
        EditorGUILayout.PropertyField(s6PitchLocalAxis);

        EditorGUILayout.Space(7f);
        GfsxSensorHeadRigController inspected =
            (GfsxSensorHeadRigController)target;
        if (!inspected.enabled)
        {
            EditorGUILayout.HelpBox(
                "Pivot editing mode is active. After moving the empty " +
                "Cam_pivot and reparenting cam_base beneath it, capture " +
                "the visible pose before enabling the controller. Do not " +
                "change the S5/S6 command sliders while editing a pivot.",
                MessageType.Info);
        }

        bool captureRequested;
        using (new EditorGUI.DisabledScope(Application.isPlaying))
        {
            captureRequested = GUILayout.Button(
                "CAPTURE CURRENT PIVOT & ENABLE");
        }

        bool propertiesChanged = serializedObject.ApplyModifiedProperties();
        if (propertiesChanged)
        {
            foreach (Object selected in targets)
            {
                GfsxSensorHeadRigController rig =
                    (GfsxSensorHeadRigController)selected;
                if (rig.enabled)
                    rig.ApplyPose();
                EditorUtility.SetDirty(rig);
                PrefabUtility.RecordPrefabInstancePropertyModifications(rig);
                if (!Application.isPlaying && rig.gameObject.scene.IsValid())
                    EditorSceneManager.MarkSceneDirty(rig.gameObject.scene);
            }

            SceneView.RepaintAll();
        }

        if (centerRequested)
        {
            foreach (Object selected in targets)
            {
                GfsxSensorHeadRigController rig =
                    (GfsxSensorHeadRigController)selected;
                Undo.RecordObject(rig, "Center GFS-X S5/S6 servos");
                rig.ResetToNeutral();
                EditorUtility.SetDirty(rig);
                PrefabUtility.RecordPrefabInstancePropertyModifications(rig);
                if (!Application.isPlaying && rig.gameObject.scene.IsValid())
                    EditorSceneManager.MarkSceneDirty(rig.gameObject.scene);
            }

            serializedObject.Update();
            SceneView.RepaintAll();
        }

        if (captureRequested)
        {
            foreach (Object selected in targets)
            {
                GfsxSensorHeadRigController rig =
                    (GfsxSensorHeadRigController)selected;
                Undo.RegisterFullObjectHierarchyUndo(
                    rig.gameObject,
                    "Capture GFS-X sensor pivot");
                Undo.RecordObject(rig, "Capture GFS-X sensor pivot");

                if (!rig.CaptureCurrentReferencePose())
                    continue;

                rig.enabled = true;
                EditorUtility.SetDirty(rig);
                PrefabUtility.RecordPrefabInstancePropertyModifications(rig);
                if (!Application.isPlaying && rig.gameObject.scene.IsValid())
                    EditorSceneManager.MarkSceneDirty(rig.gameObject.scene);
            }

            serializedObject.Update();
            SceneView.RepaintAll();
        }

        if (!inspected.IsConfigured)
        {
            EditorGUILayout.HelpBox(
                "The sensor-head rig has missing references or no captured " +
                "reference pose.",
                MessageType.Warning);
        }
    }

    private static void DrawHeader(string title)
    {
        EditorGUILayout.LabelField(title, EditorStyles.boldLabel);
    }

    private static void DrawDynamicSlider(
        SerializedProperty value,
        SerializedProperty minimum,
        SerializedProperty maximum,
        GUIContent label)
    {
        float lower = Mathf.Min(minimum.floatValue, maximum.floatValue);
        float upper = Mathf.Max(minimum.floatValue, maximum.floatValue);
        if (upper - lower < 0.001f)
            upper = lower + 0.001f;

        value.floatValue = EditorGUILayout.Slider(
            label,
            Mathf.Clamp(value.floatValue, lower, upper),
            lower,
            upper);
    }
}
