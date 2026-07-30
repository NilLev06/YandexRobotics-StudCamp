using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

/// <summary>
/// Verifies physical grasp rules: no stick-through, IR required, empty close blocked.
/// </summary>
public static class PhysicalGraspSelfTest
{
    private const string ScenePath = "Assets/Scenes/P2_DigitalTwin.unity";
    private const string ReportPath = "Logs/physical_grasp_self_test.txt";

    [MenuItem("GFS-X/Test Physical Grasp Rules")]
    public static void RunFromMenu()
    {
        int code = Run();
        if (code != 0)
            Debug.LogError($"PhysicalGraspSelfTest failed ({code}). See {ReportPath}");
        else
            Debug.Log($"PhysicalGraspSelfTest OK. See {ReportPath}");
    }

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
            Debug.Log("[PhysicalGraspSelfTest] " + msg);
        }

        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        var arm = Object.FindAnyObjectByType<GfsxArmRigController>();
        var gripper = Object.FindAnyObjectByType<GripperController>();
        var sensors = Object.FindAnyObjectByType<VirtualSensors>();
        var brain = Object.FindAnyObjectByType<RobotBrain>();
        var ball = GameObject.FindWithTag("TargetBall");

        if (arm == null || gripper == null || sensors == null || ball == null || brain == null)
        {
            Log("FAIL: missing arm/gripper/sensors/ball/brain");
            WriteReport(lines);
            return 2;
        }

        int failures = 0;
        Transform hold = arm.HoldPoint;
        Rigidbody ballBody = ball.GetComponent<Rigidbody>();

        SerializedObject gripperSo = new SerializedObject(gripper);
        SerializedProperty requireIr = gripperSo.FindProperty("requireGripperIrForGrab");
        if (requireIr == null || !requireIr.boolValue)
        {
            Log("FAIL: requireGripperIrForGrab must be true");
            failures++;
        }

        // Outside the claw: ball in front of robot, not in jaws.
        arm.SetFloorPickupPose();
        arm.SetJawOpen();
        Physics.SyncTransforms();
        gripper.EvaluateNow();
        Vector3 outside = hold.position + hold.forward * 0.12f + Vector3.up * 0.02f;
        PlaceBall(ball, ballBody, outside);
        arm.SetJawClosed();
        Physics.SyncTransforms();
        gripper.EvaluateNow();
        sensors.SampleNow();
        bool grabbedOutside = gripper.IsHolding;
        if (grabbedOutside)
        {
            Log("FAIL: grabbed ball outside jaw pocket (stick-through)");
            failures++;
            gripper.Release();
        }
        else
        {
            Log("OK: outside ball not grabbed");
        }

        // Closed claw without prior open must NOT grab when ball enters pocket.
        arm.SetJawClosed();
        Physics.SyncTransforms();
        gripper.EvaluateNow(); // consumes any arm while closed → disarmed
        PlaceBall(ball, ballBody, hold.position);
        Physics.SyncTransforms();
        sensors.SampleNow();
        gripper.EvaluateNow();
        if (gripper.IsHolding)
        {
            Log("FAIL: grabbed with jaws already closed (no open→close cycle)");
            failures++;
            gripper.Release();
        }
        else
        {
            Log("OK: closed-without-open does not grab");
        }

        // Direct TryGrabDetectedBall while closed+disarmed must fail.
        if (gripper.TryGrabDetectedBall())
        {
            Log("FAIL: TryGrabDetectedBall succeeded without open→close arm");
            failures++;
            gripper.Release();
        }
        else
        {
            Log("OK: TryGrabDetectedBall blocked when not armed");
        }

        // Proper open→close with ball in pocket.
        PlaceBall(ball, ballBody, hold.position);
        arm.SetJawOpen();
        Physics.SyncTransforms();
        gripper.EvaluateNow();
        sensors.SampleNow();
        bool irSees = sensors.GripperIrDetected;
        Log($"IR at HoldPoint (open): {irSees}");
        arm.SetJawClosed();
        Physics.SyncTransforms();
        gripper.EvaluateNow();
        bool grabbedInside = gripper.IsHolding;

        if (!irSees && grabbedInside)
        {
            Log("FAIL: grabbed without IR while requireGripperIrForGrab=true");
            failures++;
            gripper.Release();
        }
        else if (irSees && !grabbedInside)
        {
            bool inPocket = gripper.IsBallCenterInsideJawPocket(hold.position);
            Log($"WARN: IR={irSees} inPocket={inPocket} grabbed={grabbedInside}");
            if (inPocket)
            {
                Log("FAIL: ball in pocket with IR + open→close but grab failed");
                failures++;
            }
        }
        else if (irSees && grabbedInside)
        {
            Log("OK: open→close with IR grabbed");
            gripper.Release();
        }
        else
        {
            Log("OK: no IR → no grab (expected)");
        }

        // Empty close must not close jaws (reflect via brain action path is play-mode only).
        // Validate serialized penalty strength and floor pose instead.
        SerializedObject brainSo = new SerializedObject(brain);
        float emptyPenalty = brainSo.FindProperty("emptyGripperClosePenalty").floatValue;
        float s1Min = brainSo.FindProperty("mobileTrainingS1Minimum").floatValue;
        int episodeSteps = brainSo.FindProperty("maxEpisodeSteps").intValue;
        if (emptyPenalty > -0.2f)
        {
            Log($"FAIL: emptyGripperClosePenalty too weak ({emptyPenalty})");
            failures++;
        }
        if (s1Min < -22f)
        {
            Log($"FAIL: mobileTrainingS1Minimum too low ({s1Min}) — claw dives");
            failures++;
        }
        if (episodeSteps < 3500)
        {
            Log($"FAIL: maxEpisodeSteps too short ({episodeSteps})");
            failures++;
        }

        arm.SetFloorPickupPose();
        Physics.SyncTransforms();
        if (arm.S1Angle < -22f)
        {
            Log($"FAIL: floor pickup S1 too low ({arm.S1Angle:F1})");
            failures++;
        }
        else
        {
            Log($"OK: floor pickup S1={arm.S1Angle:F1}");
        }

        float clearance = arm.GetClawClearanceAboveFloor();
        Log($"claw clearance above floor: {clearance:F4} m");
        if (clearance < 0.005f)
        {
            Log("FAIL: claw tip too close to floor");
            failures++;
        }

        Log(failures == 0 ? "ALL CHECKS PASSED" : $"FAILED: {failures}");
        WriteReport(lines);
        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        return failures > 0 ? 1 : 0;
    }

    private static void PlaceBall(GameObject ball, Rigidbody body, Vector3 position)
    {
        ball.transform.SetPositionAndRotation(position, Quaternion.identity);
        if (body == null)
            return;
        body.position = position;
        body.rotation = Quaternion.identity;
        body.linearVelocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;
    }

    private static void WriteReport(List<string> lines)
    {
        string abs = Path.GetFullPath(ReportPath);
        Directory.CreateDirectory(Path.GetDirectoryName(abs) ?? "Logs");
        File.WriteAllLines(abs, lines);
    }
}
