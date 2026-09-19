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

        // Must match RobotBrain.cs serialized defaults (catch-weighted contract):
        // timePenalty 0.0008, approachRewardScale 0.7, closeApproachBoost 1.2,
        // catchSuccessReward 15, same clamping caps as EvaluateSeekRewards.
        float timePenalty = 0.0008f;
        float approachRewardScale = 0.7f;
        float closeApproachBoost = 1.2f;
        float approachMaxCap = 0.08f;
        float catchSuccessReward = 15.0f;

        // Near the ball closeness -> 1, so the per-step approach multiplier peaks
        // at scale * (1 + boost). A small approach must beat one step of time
        // penalty, otherwise the policy has no gradient to approach.
        float nearBallApproachGain = 0.01f * approachRewardScale * (1f + closeApproachBoost);
        float approachNet = Mathf.Min(nearBallApproachGain, approachMaxCap) - timePenalty;
        if (approachNet <= 0f)
        {
            log("FAIL reward-math: 1 cm approach should beat time penalty");
            failures++;
        }

        // The catch payout must dominate the dense shaping it farms (1.0 ball zone
        // reward and repeated 0.05 hold ticks), otherwise the policy loops around
        // the ball instead of committing to a grasp.
        if (catchSuccessReward <= approachRewardScale * 2f)
        {
            log("FAIL reward-math: catch success reward too weak vs approach shaping");
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

        log($"{scenePath}: mode={brain.TrainingMode} continuous={actualContinuous} obs={actualObs} (pan-only)");
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

        // Tilt is locked; agent controls pan only.
        var so = new SerializedObject(brain);
        var tiltProp = so.FindProperty("episodeCameraTiltDegrees");
        float expectedTilt = tiltProp != null ? tiltProp.floatValue : -15f;
        rig.SetPan(0f);
        rig.SetTilt(expectedTilt);
        if (Mathf.Abs(rig.S6TiltAngle - expectedTilt) > 1.5f)
        {
            log($"FAIL {scenePath}: episode tilt reset mismatch got={rig.S6TiltAngle:F1} expected={expectedTilt:F1}");
            failures++;
        }
        else
        {
            log($"{scenePath}: camera reset pan=0 tilt={rig.S6TiltAngle:F1} (tilt locked, agent pans)");
        }

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
