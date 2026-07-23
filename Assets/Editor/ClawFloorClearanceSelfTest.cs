using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>
/// FixedArm floor-pose checks. With maxClearanceRaiseDegrees=0 the calibrated
/// S1=-46 pose is authoritative (may sit close to the floor by design).
/// </summary>
public static class ClawFloorClearanceSelfTest
{
    private static readonly string[] ScenePaths =
    {
        "Assets/Scenes/P3_DigitalTwin_FixedArm.unity"
    };

    private const string ReportPath = "Logs/claw_floor_clearance_self_test.txt";
    private const float AngleEpsilon = 0.6f;

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
        var arm = Object.FindAnyObjectByType<GfsxArmRigController>(FindObjectsInactive.Include);
        var ground = GameObject.Find("Ground");

        if (arm == null || ground == null)
        {
            log($"FAIL {scenePath}: missing arm or Ground");
            return 1;
        }

        SerializedObject armSo = new SerializedObject(arm);
        float maxRaise = armSo.FindProperty("maxClearanceRaiseDegrees")?.floatValue ?? 0f;
        float expectS1 = arm.FloorPickupS1;
        float expectS2 = arm.FloorPickupS2;
        float expectS3 = arm.FloorPickupS3;

        arm.SetFloorPickupPose();
        Physics.SyncTransforms();

        if (Mathf.Abs(arm.S1Angle - expectS1) > AngleEpsilon ||
            Mathf.Abs(arm.S2Angle - expectS2) > AngleEpsilon ||
            Mathf.Abs(arm.S3Angle - expectS3) > AngleEpsilon)
        {
            log($"FAIL {scenePath}: floor pose mismatch " +
                $"got S1={arm.S1Angle:F1} S2={arm.S2Angle:F1} S3={arm.S3Angle:F1} " +
                $"expected S1={expectS1:F1} S2={expectS2:F1} S3={expectS3:F1}");
            failures++;
        }
        else
        {
            log($"OK {scenePath}: floor pose S1={arm.S1Angle:F1} S2={arm.S2Angle:F1} S3={arm.S3Angle:F1}");
        }

        if (Mathf.Abs(expectS1 - (-46f)) > AngleEpsilon)
        {
            log($"FAIL {scenePath}: FloorPickupS1 must be -46 (got {expectS1:F1})");
            failures++;
        }

        float clearance = arm.GetClawClearanceAboveFloor();
        log($"{scenePath}: floor_pickup clearance={clearance:F4}m " +
            $"(maxClearanceRaiseDegrees={maxRaise:F1})");

        if (maxRaise > 0.01f)
        {
            float minClearance = arm.MinTipClearanceMetres;
            if (clearance + 1e-4f < minClearance)
            {
                log($"FAIL {scenePath}: clearance {clearance:F4} < required {minClearance:F4}");
                failures++;
            }
        }
        else
        {
            // Strict calibrated pose: allow near-floor; only flag deep penetration.
            if (clearance < -0.02f)
            {
                log($"FAIL {scenePath}: claw deeply penetrating floor ({clearance:F4}m)");
                failures++;
            }
            else
            {
                log($"OK {scenePath}: strict S1=-46 mode (clearance raise disabled)");
            }
        }

        // Chassis must still collide with Ground.
        var groundCollider = ground.GetComponent<Collider>();
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
