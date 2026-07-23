using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>
/// Batch / menu self-test: finds arm angles where HoldPoint can physically
/// overlap a Ø5 cm TargetBall and GripperController.TryGrab succeeds.
/// </summary>
public static class ClawGraspSelfTest
{
    private const string ScenePath = "Assets/Scenes/P3_DigitalTwin_FixedArm.unity";
    private const string ReportPath = "Logs/claw_grasp_self_test.txt";
    private const float BallDiameter = 0.05f;
    private const float BallRadius = BallDiameter * 0.5f;

    [MenuItem("GFS-X/Test Claw Grasp Poses")]
    public static void RunFromMenu()
    {
        int code = Run();
        if (code != 0)
            Debug.LogError($"ClawGraspSelfTest failed with code {code}. See {ReportPath}");
        else
            Debug.Log($"ClawGraspSelfTest OK. See {ReportPath}");
    }

    /// <summary>Unity batchmode entry: -executeMethod ClawGraspSelfTest.RunBatch</summary>
    public static void RunBatch()
    {
        EditorApplication.Exit(Run());
    }

    public static int Run()
    {
        var lines = new List<string>();
        void Log(string msg)
        {
            lines.Add(msg);
            Debug.Log("[ClawGraspSelfTest] " + msg);
        }

        var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        var arm = Object.FindAnyObjectByType<GfsxArmRigController>();
        var gripper = Object.FindAnyObjectByType<GripperController>();
        var ball = GameObject.FindWithTag("TargetBall");

        if (arm == null || gripper == null || ball == null)
        {
            Log("FAIL: missing arm / gripper / TargetBall");
            WriteReport(lines);
            return 2;
        }

        var ballBody = ball.GetComponent<Rigidbody>();
        var hold = arm.HoldPoint;
        if (hold == null || ballBody == null)
        {
            Log("FAIL: missing HoldPoint or ball Rigidbody");
            WriteReport(lines);
            return 2;
        }

        float graspRadius = gripper.GraspRadius;
        Log($"graspRadius={graspRadius:F3} ballRadius={BallRadius:F3}");
        Log($"limits S1=[{arm.S1MinimumAngle:F0},{arm.S1MaximumAngle:F0}] " +
            $"S2=[{arm.S2MinimumAngle:F0},{arm.S2MaximumAngle:F0}]");

        // Geometric hold-point sweep: temporarily allow grab without IR so we
        // can validate jaw pocket geometry independently of sensor noise.
        var gripperSo = new SerializedObject(gripper);
        var requireIr = gripperSo.FindProperty("requireGripperIrForGrab");
        bool previousRequireIr = requireIr != null && requireIr.boolValue;
        if (requireIr != null)
        {
            requireIr.boolValue = false;
            gripperSo.ApplyModifiedPropertiesWithoutUndo();
        }

        // Candidate poses: current defaults + sweep around floor-reachable set.
        var candidates = new List<(float s1, float s2, float s3, string name)>
        {
            (0f, 0f, 0f, "neutral"),
            (0f, -45f, 0f, "prev_horizontal"),
            (-30f, -45f, -90f, "old_floor_pickup"),
            (-35f, -11f, 0f, "dives_into_ball"),
            (-35f, -20f, 90f, "parallel_floor_fix"),
            (-34f, -12f, 0f, "fk_best_horizontal"),
            (-35f, -11f, 0f, "fk_alt_a"),
            (-32f, -14f, 0f, "fk_alt_b"),
            (-20f, -25f, 0f, "mid"),
            (-10f, -35f, 0f, "mid2"),
        };

        // Dense sweep within configured servo limits.
        for (float s1 = arm.S1MinimumAngle; s1 <= arm.S1MaximumAngle; s1 += 5f)
        {
            for (float s2 = arm.S2MinimumAngle; s2 <= arm.S2MaximumAngle; s2 += 5f)
                candidates.Add((s1, s2, 0f, $"sweep_{s1:F0}_{s2:F0}"));
        }

        int geometricHits = 0;
        int grabHits = 0;
        string best = null;
        float bestScore = float.PositiveInfinity;

        // Restore ball after each try.
        Vector3 ballStart = ball.transform.position;
        Quaternion ballStartRot = ball.transform.rotation;

        foreach (var (s1, s2, s3, name) in candidates)
        {
            if (gripper.IsHolding)
                gripper.Release();

            arm.SetServoCommands(s1, s2, s3, arm.S4MinimumClosureLimit);
            Physics.SyncTransforms();

            // Place ball so its center sits at HoldPoint — isolates whether
            // the claw orientation/volume can capture, independent of driving.
            Vector3 holdPos = hold.position;
            ball.transform.SetPositionAndRotation(holdPos, Quaternion.identity);
            if (ballBody != null)
            {
                ballBody.position = holdPos;
                ballBody.rotation = Quaternion.identity;
                ballBody.linearVelocity = Vector3.zero;
                ballBody.angularVelocity = Vector3.zero;
            }

            Physics.SyncTransforms();

            float dist = Vector3.Distance(hold.position, ball.transform.position);
            bool overlap = dist <= graspRadius + BallRadius + 1e-4f;

            // Proper open→close cycle (grab is edge-triggered).
            arm.SetJawOpen();
            Physics.SyncTransforms();
            gripper.EvaluateNow();
            arm.SetJawClosed();
            Physics.SyncTransforms();
            gripper.EvaluateNow();
            bool grabbed = gripper.IsHolding;

            Vector3 fwd = hold.forward;
            float pitch = Mathf.Asin(Mathf.Clamp(fwd.y, -1f, 1f)) * Mathf.Rad2Deg;
            float score = Mathf.Abs(pitch) + (grabbed ? 0f : 100f);

            if (overlap)
                geometricHits++;
            if (grabbed)
            {
                grabHits++;
                if (score < bestScore)
                {
                    bestScore = score;
                    best = $"{name}: S1={s1:F1} S2={s2:F1} S3={s3:F1} " +
                           $"pitch={pitch:F1} holdY={holdPos.y:F3} grabbed=YES";
                }
            }

            if (name.StartsWith("fk_") ||
                name is "neutral" or "prev_horizontal" or "old_floor_pickup"
                    or "mid" or "mid2" or "dives_into_ball" or "parallel_floor_fix")
            {
                Log($"{name}: S1={s1:F0} S2={s2:F0} S3={s3:F0} hold=({holdPos.x:F3},{holdPos.y:F3},{holdPos.z:F3}) " +
                    $"pitch={pitch:F1} overlap={overlap} grabbed={grabbed}");
            }

            if (gripper.IsHolding)
                gripper.Release();

            ball.transform.SetPositionAndRotation(ballStart, ballStartRot);
            if (ballBody != null)
            {
                ballBody.position = ballStart;
                ballBody.rotation = ballStartRot;
            }
        }

        // Also test: scene ball at its authored position with best horizontal pose.
        Log("--- scene-ball reachability (no teleport) ---");
        foreach (var (s1, s2, s3, name) in new[]
                 {
                     (-35f, -20f, 90f, "parallel_floor_fix"),
                     (-35f, -11f, 0f, "dives_into_ball"),
                     (0f, -45f, 0f, "prev_horizontal"),
                     (0f, 0f, 0f, "neutral"),
                 })
        {
            arm.SetServoCommands(s1, s2, s3, arm.S4MinimumClosureLimit);
            Physics.SyncTransforms();
            float d = Vector3.Distance(hold.position, ballStart);
            float surface = Mathf.Max(0f, d - BallRadius);
            bool can = surface <= graspRadius;
            Log($"scene@{name}: holdDist={d:F3} surface={surface:F3} canGraspVolume={can} " +
                $"hold=({hold.position.x:F3},{hold.position.y:F3},{hold.position.z:F3})");
        }

        Log($"summary: geometricHits={geometricHits} grabHits={grabHits} candidates={candidates.Count}");
        Log(best != null ? "BEST " + best : "BEST none");

        // Restore original scene objects; do not save — caller applies pose after review.
        if (gripper.IsHolding)
            gripper.Release();
        arm.SetServoCommands(-35f, -20f, 90f, arm.S4MinimumClosureLimit);
        arm.SetJawOpen();
        ball.transform.SetPositionAndRotation(ballStart, ballStartRot);
        if (ballBody != null)
        {
            ballBody.position = ballStart;
            ballBody.rotation = ballStartRot;
        }

        WriteReport(lines);
        // Discard pose mutations from the sweep.
        if (requireIr != null)
        {
            requireIr.boolValue = previousRequireIr;
            gripperSo.ApplyModifiedPropertiesWithoutUndo();
        }

        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);

        return grabHits > 0 ? 0 : 1;
    }

    private static void WriteReport(List<string> lines)
    {
        string abs = Path.GetFullPath(ReportPath);
        Directory.CreateDirectory(Path.GetDirectoryName(abs) ?? "Logs");
        File.WriteAllLines(abs, lines);
    }
}
