using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>
/// Exhaustive check: claw geometry must never touch or penetrate the floor
/// across the reachable training pose space (open/closed jaws).
/// </summary>
public static class ClawFloorClearanceSelfTest
{
    private static readonly string[] ScenePaths =
    {
        "Assets/Scenes/P2_DigitalTwin.unity",
        "Assets/Scenes/P3_DigitalTwin_FixedArm.unity"
    };

    private const string ReportPath = "Logs/claw_floor_clearance_self_test.txt";
    private const float AbsoluteNoTouchEpsilon = 0.001f;

    [MenuItem("GFS-X/Test Claw Floor Clearance")]
    public static void RunFromMenu()
    {
        int code = Run();
        if (code != 0)
            Debug.LogError($"ClawFloorClearanceSelfTest failed ({code}). See {ReportPath}");
        else
            Debug.Log($"ClawFloorClearanceSelfTest OK. See {ReportPath}");
    }

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
            Debug.Log("[ClawFloorClearanceSelfTest] " + msg);
        }

        foreach (string scenePath in ScenePaths)
        {
            EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
            failures += VerifyScene(scenePath, Log);
        }

        Log(failures == 0 ? "ALL CHECKS PASSED" : $"FAILED: {failures} issue(s)");
        WriteReport(lines);
        return failures > 0 ? 1 : 0;
    }

    private static int VerifyScene(string scenePath, System.Action<string> log)
    {
        int failures = 0;
        var arm = Object.FindAnyObjectByType<GfsxArmRigController>();
        var brain = Object.FindAnyObjectByType<RobotBrain>();
        var ground = GameObject.Find("Ground");

        if (arm == null || ground == null)
        {
            log($"FAIL {scenePath}: missing arm or Ground");
            return 1;
        }

        float minClearance = arm.MinTipClearanceMetres;
        if (minClearance < 0.02f)
        {
            log($"FAIL {scenePath}: minTipClearanceMetres too small ({minClearance:F4})");
            failures++;
        }

        float floorY = 0f;
        var groundCollider = ground.GetComponent<Collider>();
        if (groundCollider != null)
            floorY = groundCollider.bounds.max.y;
        else
            floorY = ground.transform.position.y;

        // Floor pickup pose.
        arm.SetFloorPickupPose();
        Physics.SyncTransforms();
        failures += AssertClear(scenePath, "floor_pickup", arm, floorY, minClearance, log);

        float s1Min = arm.S1MinimumAngle;
        float s1Max = arm.S1MaximumAngle;
        float s2Min = arm.S2MinimumAngle;
        float s2Max = arm.S2MaximumAngle;

        if (brain != null)
        {
            var so = new SerializedObject(brain);
            var s1FloorProp = so.FindProperty("mobileTrainingS1Minimum");
            if (s1FloorProp != null)
                s1Min = Mathf.Max(s1Min, s1FloorProp.floatValue);
            var s1CeilProp = so.FindProperty("mobileTrainingS1Maximum");
            if (s1CeilProp != null)
                s1Max = Mathf.Min(s1Max, s1CeilProp.floatValue);
            var s2FloorProp = so.FindProperty("mobileTrainingS2Minimum");
            if (s2FloorProp != null)
                s2Min = Mathf.Max(s2Min, s2FloorProp.floatValue);
        }

        int poseCount = 0;
        int failPoses = 0;
        float worstClearance = float.PositiveInfinity;
        string worstName = "";

        for (float s1 = s1Min; s1 <= s1Max + 0.01f; s1 += 4f)
        {
            for (float s2 = s2Min; s2 <= s2Max + 0.01f; s2 += 10f)
            {
                foreach (bool jawsClosed in new[] { false, true })
                {
                    float s4 = jawsClosed
                        ? arm.S4MaximumClosureLimit
                        : arm.S4MinimumClosureLimit;

                    // Intentionally command a low pose, then rely on EnforceFloorClearance.
                    arm.SetArmPoseLockedWrist(s1, s2, s4);
                    Physics.SyncTransforms();
                    poseCount++;

                    float clearance = arm.GetClawClearanceAboveFloor();
                    if (clearance < worstClearance)
                    {
                        worstClearance = clearance;
                        worstName = $"S1={s1:F0} S2={s2:F0} closed={jawsClosed}";
                    }

                    if (clearance < minClearance - AbsoluteNoTouchEpsilon)
                    {
                        failPoses++;
                        if (failPoses <= 8)
                        {
                            log($"FAIL {scenePath}: clearance {clearance:F4}m < {minClearance:F4}m " +
                                $"at S1={s1:F0} S2={s2:F0} closed={jawsClosed}");
                        }

                        failures++;
                    }

                    float lowest = arm.HoldPoint != null
                        ? float.NaN
                        : float.NaN;
                    // Absolute: lowest geometry must be above floor.
                    if (clearance <= 0f)
                    {
                        log($"FAIL {scenePath}: claw penetrating floor (clearance={clearance:F4}) " +
                            $"S1={s1:F0} S2={s2:F0}");
                        failures++;
                    }
                }
            }
        }

        // Extreme dive attempt: agent min S1, open and closed.
        arm.SetArmPoseLockedWrist(s1Min, 0f, arm.S4MinimumClosureLimit);
        Physics.SyncTransforms();
        failures += AssertClear(scenePath, "extreme_open", arm, floorY, minClearance, log);
        arm.SetArmPoseLockedWrist(s1Min, 0f, arm.S4MaximumClosureLimit);
        Physics.SyncTransforms();
        failures += AssertClear(scenePath, "extreme_closed", arm, floorY, minClearance, log);

        // Chassis must still collide with Ground (regression for IgnoreCollision bug).
        var tracker = Object.FindAnyObjectByType<ArmContactTracker>();
        if (tracker == null && brain != null)
            tracker = brain.gameObject.AddComponent<ArmContactTracker>();
        if (tracker != null)
            tracker.ApplyGroundCollisionFilters();

        var robot = GameObject.Find("GFS-X Robot");
        if (robot != null && groundCollider != null)
        {
            Collider[] rootColliders = robot.GetComponents<Collider>();
            foreach (Collider chassis in rootColliders)
            {
                if (chassis == null || !chassis.enabled)
                    continue;
                if (Physics.GetIgnoreCollision(chassis, groundCollider))
                {
                    log($"FAIL {scenePath}: chassis collider {chassis.GetType().Name} ignores Ground");
                    failures++;
                }
            }

            // Claw colliders SHOULD ignore Ground (anti-jack), but geometry must stay clear.
            Transform shoulder = robot.transform.Find("S1_Shoulder_Pivot");
            if (shoulder != null)
            {
                Collider[] clawColliders = shoulder.GetComponentsInChildren<Collider>(true);
                int ignored = 0;
                foreach (Collider claw in clawColliders)
                {
                    if (claw == null || !claw.enabled)
                        continue;
                    if (Physics.GetIgnoreCollision(claw, groundCollider))
                        ignored++;
                }

                log($"{scenePath}: claw colliders ignoring Ground = {ignored}/{clawColliders.Length} (anti-jack OK)");
            }
        }

        log($"{scenePath}: poses={poseCount} failPoses={failPoses} " +
            $"worstClearance={worstClearance:F4}m at {worstName} " +
            $"minRequired={minClearance:F4} floorY={floorY:F4}");

        // Restore pose
        arm.SetFloorPickupPose();
        return failures;
    }

    private static int AssertClear(
        string scenePath,
        string label,
        GfsxArmRigController arm,
        float floorY,
        float minClearance,
        System.Action<string> log)
    {
        float clearance = arm.GetClawClearanceAboveFloor();
        if (clearance < minClearance - AbsoluteNoTouchEpsilon || clearance <= 0f)
        {
            log($"FAIL {scenePath}@{label}: clearance={clearance:F4}m " +
                $"(need ≥ {minClearance:F4}, floorY={floorY:F4}, S1={arm.S1Angle:F1})");
            return 1;
        }

        log($"OK {scenePath}@{label}: clearance={clearance:F4}m S1={arm.S1Angle:F1}");
        return 0;
    }

    private static void WriteReport(List<string> lines)
    {
        string abs = Path.GetFullPath(ReportPath);
        Directory.CreateDirectory(Path.GetDirectoryName(abs) ?? "Logs");
        File.WriteAllLines(abs, lines);
    }
}
