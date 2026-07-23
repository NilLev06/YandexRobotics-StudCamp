using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>
/// Verifies chassis colliders still collide with Ground while claw colliders do not.
/// </summary>
public static class ArmContactTrackerSelfTest
{
    private static readonly string[] ScenePaths =
    {
        "Assets/Scenes/P3_DigitalTwin_FixedArm.unity"
    };
    private const string ReportPath = "Logs/arm_contact_tracker_self_test.txt";

    [MenuItem("GFS-X/Test Arm Ground Collisions")]
    public static void RunFromMenu()
    {
        int code = Run();
        if (code != 0)
            Debug.LogError($"ArmContactTrackerSelfTest failed with code {code}. See {ReportPath}");
        else
            Debug.Log($"ArmContactTrackerSelfTest OK. See {ReportPath}");
    }

    /// <summary>Unity batchmode entry: -executeMethod ArmContactTrackerSelfTest.RunBatch</summary>
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
            Debug.Log("[ArmContactTrackerSelfTest] " + msg);
        }

        int failures = 0;
        foreach (string scenePath in ScenePaths)
        {
            EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
            failures += VerifyScene(scenePath, Log);
        }

        WriteReport(lines);
        return failures > 0 ? 1 : 0;
    }

    private static int VerifyScene(string scenePath, System.Action<string> log)
    {
        int failures = 0;
        var robot = GameObject.Find("GFS-X Robot");
        var ground = GameObject.Find("Ground");
        if (robot == null || ground == null)
        {
            log($"FAIL {scenePath}: missing robot or ground");
            return 1;
        }

        var tracker = robot.GetComponent<ArmContactTracker>();
        if (tracker == null)
            tracker = robot.AddComponent<ArmContactTracker>();
        else
        {
            Object.DestroyImmediate(tracker);
            tracker = robot.AddComponent<ArmContactTracker>();
        }

        tracker.ApplyGroundCollisionFilters();

        Collider[] robotColliders = robot.GetComponentsInChildren<Collider>(true);
        Collider[] groundColliders = ground.GetComponentsInChildren<Collider>(true);
        if (groundColliders.Length == 0)
        {
            log($"FAIL {scenePath}: ground has no colliders");
            return 1;
        }

        int chassisColliders = 0;
        int armColliders = 0;
        int chassisIgnored = 0;
        int armIgnored = 0;

        Transform shoulder = robot.transform.Find("S1_Shoulder_Pivot");
        foreach (Collider robotCollider in robotColliders)
        {
            if (robotCollider == null || !robotCollider.enabled)
                continue;

            bool isArm = shoulder != null &&
                         (robotCollider.transform == shoulder ||
                          robotCollider.transform.IsChildOf(shoulder));
            bool isChassis = robotCollider.transform == robot.transform;

            if (isChassis)
                chassisColliders++;
            if (isArm)
                armColliders++;

            foreach (Collider groundCollider in groundColliders)
            {
                if (groundCollider == null || !groundCollider.enabled)
                    continue;

                bool ignored = Physics.GetIgnoreCollision(robotCollider, groundCollider);
                if (isChassis && ignored)
                {
                    chassisIgnored++;
                    log($"FAIL {scenePath}: chassis collider {robotCollider.name} ignores Ground");
                    failures++;
                }

                if (isArm && !ignored)
                {
                    log($"FAIL {scenePath}: arm collider {robotCollider.name} still collides with Ground");
                    failures++;
                }

                if (isArm && ignored)
                    armIgnored++;
            }
        }

        if (chassisColliders == 0)
        {
            log($"FAIL {scenePath}: no chassis colliders found on robot root");
            failures++;
        }

        if (armColliders == 0)
        {
            log($"FAIL {scenePath}: no arm colliders found under {ShoulderPivotName()}");
            failures++;
        }

        log($"{scenePath}: chassis={chassisColliders} arm={armColliders} " +
            $"chassisIgnored={chassisIgnored} armIgnoredPairs={armIgnored} failures={failures}");
        return failures;
    }

    private static string ShoulderPivotName() => "S1_Shoulder_Pivot";

    private static void WriteReport(List<string> lines)
    {
        string abs = Path.GetFullPath(ReportPath);
        Directory.CreateDirectory(Path.GetDirectoryName(abs) ?? "Logs");
        File.WriteAllLines(abs, lines);
    }
}
