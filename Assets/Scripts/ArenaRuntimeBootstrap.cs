using Unity.MLAgents;
using Unity.MLAgents.Policies;
using UnityEngine;

/// <summary>
/// If the scene was not wrapped by the editor menu yet, groups root arena objects
/// under Arena_0 at play time and clones 40 fixed-size training fields.
/// </summary>
public static class ArenaRuntimeBootstrap
{
    private const int DefaultArenaCount = 40;
    private const int DefaultColumns = 8;
    private static readonly Vector2 DefaultSpacing = new Vector2(9f, 7f);
    private static readonly Vector2 PlayableHalfExtents = new Vector2(2.8f, 2.0f);

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    private static void AfterSceneLoad()
    {
        if (Object.FindAnyObjectByType<TrainingArena>() != null)
        {
            EnsureManagerOnly();
            return;
        }

        GameObject robot = GameObject.Find("GFS-X Robot");
        GameObject ground = GameObject.Find("Ground");
        GameObject ball = GameObject.Find("TargetBall");
        if (robot == null || ground == null || ball == null)
        {
            Debug.LogWarning("ArenaRuntimeBootstrap: arena roots not found, skip.");
            return;
        }

        GameObject arenaObject = new GameObject("Arena_0");
        Parent(ground, arenaObject);
        Parent(ball, arenaObject);
        Parent(robot, arenaObject);
        ParentIfExists("Cube", arenaObject);
        ParentIfExists("Cube (1)", arenaObject);
        ParentIfExists("Cube (2)", arenaObject);
        ParentIfExists("Cube (3)", arenaObject);

        TrainingArena arena = arenaObject.AddComponent<TrainingArena>();
        RobotBrain brain = robot.GetComponent<RobotBrain>();
        SimulatedYoloCamera camera = robot.GetComponentInChildren<SimulatedYoloCamera>(true);
        arena.Configure(brain, ball.transform, camera, PlayableHalfExtents);
        if (brain != null)
            brain.BindTrainingArena(arena);

        GameObject managerObject = new GameObject("MultiArenaManager");
        MultiArenaManager manager = managerObject.AddComponent<MultiArenaManager>();
        int arenaCount = ResolveArenaCount();
        manager.Configure(arena, arenaCount, DefaultColumns, DefaultSpacing);
        manager.BuildArenas();

        Debug.Log($"ArenaRuntimeBootstrap: {arenaCount} fixed-size arenas ready.");
    }

    private static void EnsureManagerOnly()
    {
        TrainingArena template = Object.FindAnyObjectByType<TrainingArena>();
        if (template == null)
            return;

        MultiArenaManager manager = Object.FindAnyObjectByType<MultiArenaManager>();
        if (manager == null)
        {
            GameObject managerObject = new GameObject("MultiArenaManager");
            manager = managerObject.AddComponent<MultiArenaManager>();
        }

        manager.Configure(template, ResolveArenaCount(), DefaultColumns, DefaultSpacing);
        manager.BuildArenas();
    }

    private static int ResolveArenaCount()
    {
        // Prefer env override so headless training never silently falls to 1 arena
        // if Visual Validation left InferenceOnly in the scene.
        string envCount = System.Environment.GetEnvironmentVariable("GFSX_ARENA_COUNT");
        if (!string.IsNullOrEmpty(envCount) && int.TryParse(envCount, out int parsed) && parsed > 0)
            return parsed;

        // AfterSceneLoad may run before the communicator is ready.
        // Headless workers launched by mlagents-learn always get --port.
        if (HasMlAgentsPortArg())
            return DefaultArenaCount;

        bool trainingWithPython = Academy.IsInitialized && Academy.Instance.IsCommunicatorOn;
        if (trainingWithPython)
            return DefaultArenaCount;

        RobotBrain[] agents = Object.FindObjectsByType<RobotBrain>(
            FindObjectsInactive.Include,
            FindObjectsSortMode.None);
        foreach (RobotBrain agent in agents)
        {
            BehaviorParameters behavior = agent.GetComponent<BehaviorParameters>();
            if (behavior != null && behavior.BehaviorType == BehaviorType.InferenceOnly)
                return 1;
        }

        return DefaultArenaCount;
    }

    private static bool HasMlAgentsPortArg()
    {
        string[] args = System.Environment.GetCommandLineArgs();
        for (int i = 0; i < args.Length; i++)
        {
            if (args[i] == "--port" || args[i] == "--mlagents-port")
                return true;
        }

        return false;
    }

    private static void ParentIfExists(string name, GameObject parent)
    {
        GameObject child = GameObject.Find(name);
        if (child != null)
            Parent(child, parent);
    }

    private static void Parent(GameObject child, GameObject parent)
    {
        child.transform.SetParent(parent.transform, true);
    }
}
