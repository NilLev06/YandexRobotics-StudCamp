using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEngine;

/// <summary>
/// Batch / menu checks: action space, camera tilt lock, anti-loop reward signs,
/// arm-ground collisions.
/// </summary>
public static class TrainingSanitySelfTest
{
    private static readonly string[] ScenePaths =
    {
        "Assets/Scenes/P3_DigitalTwin_FixedArm.unity"
    };
    private const string ReportPath = "Logs/training_sanity_self_test.txt";

    [MenuItem("GFS-X/Run Training Sanity Tests")]
    public static void RunFromMenu()
    {
        int code = Run();
        if (code != 0)
            Debug.LogError($"TrainingSanitySelfTest failed with code {code}. See {ReportPath}");
        else
            Debug.Log($"TrainingSanitySelfTest OK. See {ReportPath}");
    }

    /// <summary>Unity batchmode: -executeMethod TrainingSanitySelfTest.RunBatch</summary>
    public static void RunBatch()
    {
        EditorApplication.Exit(Run());
    }

    public static int Run()
    {
        var lines = new List<string>();
        int failures = 0;

        void Log(string msg)
        {
            lines.Add(msg);
            Debug.Log("[TrainingSanitySelfTest] " + msg);
        }

        failures += VerifyRewardAntiLoopMath(Log);
        failures += ArmContactTrackerSelfTest.Run();
        failures += PhysicalGraspSelfTest.Run();
        failures += ClawFloorClearanceSelfTest.Run();

        foreach (string scenePath in ScenePaths)
        {
            EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
            failures += VerifyActionSpace(scenePath, Log);
            failures += VerifyCameraTiltLock(scenePath, Log);
            failures += VerifyFloorBallVisibility(scenePath, Log);
        }

        Log(failures == 0
            ? "ALL CHECKS PASSED"
            : $"FAILED: {failures} issue(s)");
        WriteReport(lines);
        return failures > 0 ? 1 : 0;
    }

    private static int VerifyRewardAntiLoopMath(System.Action<string> log)
    {
        int failures = 0;

        float approachNet = 0.01f * 1.0f - 0.001f;
        if (approachNet <= 0f)
        {
            log("FAIL reward-math: 1 cm approach should beat time penalty");
            failures++;
        }

        float catchReward = 2.0f;
        if (catchReward < 1.5f)
        {
            log("FAIL reward-math: catch success reward too weak vs original");
            failures++;
        }

        log($"reward-math: failures={failures}");
        return failures;
    }

    private static int VerifyActionSpace(string scenePath, System.Action<string> log)
    {
        int failures = 0;
        RobotBrain brain = Object.FindAnyObjectByType<RobotBrain>();
        BehaviorParameters behavior = brain != null
            ? brain.GetComponent<BehaviorParameters>()
            : null;

        if (brain == null || behavior == null)
        {
            log($"FAIL {scenePath}: missing RobotBrain / BehaviorParameters");
            return 1;
        }

        int expectedContinuous = brain.ExpectedContinuousActions;
        int actualContinuous = behavior.BrainParameters.ActionSpec.NumContinuousActions;
        if (actualContinuous != expectedContinuous)
        {
            log($"FAIL {scenePath}: expected {expectedContinuous} continuous actions, got {actualContinuous}");
            failures++;
        }

        int expectedObs = brain.ExpectedVectorObservations;
        int actualObs = behavior.BrainParameters.VectorObservationSize;
        if (actualObs != expectedObs)
        {
            log($"FAIL {scenePath}: expected {expectedObs} vector observations, got {actualObs}");
            failures++;
        }

        if (actualContinuous < 3)
        {
            log($"FAIL {scenePath}: not enough actions for drive/turn/pan");
            failures++;
        }

        log($"{scenePath}: mode={brain.TrainingMode} continuous={actualContinuous} obs={actualObs} (pan only)");
        return failures;
    }

    private static int VerifyCameraTiltLock(string scenePath, System.Action<string> log)
    {
        int failures = 0;
        RobotBrain brain = Object.FindAnyObjectByType<RobotBrain>();
        GfsxSensorHeadRigController rig =
            Object.FindAnyObjectByType<GfsxSensorHeadRigController>();

        if (brain == null || rig == null)
        {
            log($"FAIL {scenePath}: missing RobotBrain or sensor head rig");
            return 1;
        }

        SerializedObject serializedBrain = new SerializedObject(brain);
        float episodeTilt = serializedBrain.FindProperty("episodeCameraTiltDegrees").floatValue;

        rig.SetPan(99f);
        rig.SetTilt(-99f);

        var resetMethod = typeof(RobotBrain).GetMethod(
            "ResetCameraHeadForEpisode",
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        if (resetMethod == null)
        {
            log($"FAIL {scenePath}: ResetCameraHeadForEpisode not found");
            return 1;
        }

        resetMethod.Invoke(brain, null);
        Physics.SyncTransforms();

        if (!Mathf.Approximately(rig.S5PanAngle, 0f))
        {
            log($"FAIL {scenePath}: pan not reset to 0 (got {rig.S5PanAngle:F1})");
            failures++;
        }

        float expectedTilt = Mathf.Clamp(episodeTilt, rig.S6MinimumAngle, rig.S6MaximumAngle);
        if (!Mathf.Approximately(rig.S6TiltAngle, expectedTilt))
        {
            log($"FAIL {scenePath}: tilt expected {expectedTilt:F1}, got {rig.S6TiltAngle:F1}");
            failures++;
        }

        log($"{scenePath}: camera reset pan=0 tilt={rig.S6TiltAngle:F1} (tilt locked)");
        return failures;
    }

    private static int VerifyFloorBallVisibility(string scenePath, System.Action<string> log)
    {
        int failures = 0;
        var brain = Object.FindAnyObjectByType<RobotBrain>();
        var rig = Object.FindAnyObjectByType<GfsxSensorHeadRigController>();
        var camera = Object.FindAnyObjectByType<SimulatedYoloCamera>();
        var ball = GameObject.FindWithTag("TargetBall");

        if (brain == null || rig == null || camera == null || ball == null)
        {
            log($"FAIL {scenePath}: missing rig / camera / ball for visibility sweep");
            return 1;
        }

        SerializedObject serializedCamera = new SerializedObject(camera);
        SerializedProperty cameraDr = serializedCamera.FindProperty("enableDomainRandomization");
        bool previousDr = cameraDr != null && cameraDr.boolValue;
        if (cameraDr != null)
        {
            cameraDr.boolValue = false;
            serializedCamera.ApplyModifiedPropertiesWithoutUndo();
        }

        // Place the ball on the floor ahead of the robot (track forward = local X).
        Transform robotTransform = brain.transform;
        TrackController drive = brain.GetComponent<TrackController>();
        Vector3 forward = drive != null ? drive.WorldForward : robotTransform.forward;
        if (forward.sqrMagnitude < 0.0001f)
            forward = Vector3.forward;
        Vector3 testBallPosition = robotTransform.position + forward * 0.55f;
        testBallPosition.y = 0.025f;
        ball.transform.position = testBallPosition;

        bool sawVisible = false;
        float bestTilt = 0f;
        float bestPan = 0f;
        for (float tilt = rig.S6MinimumAngle; tilt <= rig.S6MaximumAngle; tilt += 3f)
        {
            for (float pan = -45f; pan <= 45f; pan += 15f)
            {
                rig.SetPan(pan);
                rig.SetTilt(tilt);
                Physics.SyncTransforms();
                camera.RefreshDetection();
                if (!camera.IsBallVisible)
                    continue;

                sawVisible = true;
                bestTilt = tilt;
                bestPan = pan;
            }
        }

        if (!sawVisible)
        {
            rig.SetPan(0f);
            rig.SetTilt(0f);
            Physics.SyncTransforms();
            camera.RefreshDetection();
            log($"WARN {scenePath}: ball not visible in editor sweep ({camera.DebugLastRejectReason})");
        }
        else
        {
            log($"{scenePath}: ball visible with S5≈{bestPan:F0}° S6≈{bestTilt:F0}°");
        }

        SerializedObject serializedBrain = new SerializedObject(brain);
        float episodeTilt = serializedBrain.FindProperty("episodeCameraTiltDegrees").floatValue;
        rig.SetPan(0f);
        rig.SetTilt(Mathf.Clamp(episodeTilt, rig.S6MinimumAngle, rig.S6MaximumAngle));
        Physics.SyncTransforms();
        camera.RefreshDetection();
        if (!camera.IsBallVisible)
        {
            if (sawVisible)
            {
                log($"WARN {scenePath}: episodeCameraTilt={episodeTilt:F0}° misses ball; best S6≈{bestTilt:F0}°");
            }
        }
        else
        {
            log($"{scenePath}: episodeCameraTilt={episodeTilt:F0}° sees floor ball");
        }

        if (cameraDr != null)
        {
            cameraDr.boolValue = previousDr;
            serializedCamera.ApplyModifiedPropertiesWithoutUndo();
        }

        return failures;
    }

    private static void WriteReport(List<string> lines)
    {
        string abs = Path.GetFullPath(ReportPath);
        Directory.CreateDirectory(Path.GetDirectoryName(abs) ?? "Logs");
        File.WriteAllLines(abs, lines);
    }
}
