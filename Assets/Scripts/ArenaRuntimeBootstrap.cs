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
        if (Object.FindFirstObjectByType<TrainingArena>() != null)
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

        GameObject managerObject = new GameObject("MultiArenaManager");
        MultiArenaManager manager = managerObject.AddComponent<MultiArenaManager>();
        manager.Configure(arena, DefaultArenaCount, DefaultColumns, DefaultSpacing);
        manager.BuildArenas();

        Debug.Log($"ArenaRuntimeBootstrap: {DefaultArenaCount} fixed-size arenas ready.");
    }

    private static void EnsureManagerOnly()
    {
        TrainingArena template = Object.FindFirstObjectByType<TrainingArena>();
        if (template == null)
            return;

        MultiArenaManager manager = Object.FindFirstObjectByType<MultiArenaManager>();
        if (manager == null)
        {
            GameObject managerObject = new GameObject("MultiArenaManager");
            manager = managerObject.AddComponent<MultiArenaManager>();
        }

        manager.Configure(template, DefaultArenaCount, DefaultColumns, DefaultSpacing);
        manager.BuildArenas();
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
