using Unity.MLAgents;

/// <summary>
/// TensorBoard metric: percent of agent episodes that caught the ball (0–100).
/// </summary>
public static class TrainingGraspMetrics
{
    private static int totalEpisodes;
    private static int totalCatches;

    public static int TotalEpisodes => totalEpisodes;
    public static int TotalCatches => totalCatches;

    public static float CatchPercent =>
        totalEpisodes > 0 ? 100f * totalCatches / totalEpisodes : 0f;

    public static void ResetSession()
    {
        totalEpisodes = 0;
        totalCatches = 0;
    }

    public static void RegisterEpisodeEnd(bool caughtBall)
    {
        totalEpisodes++;
        if (caughtBall)
            totalCatches++;

        if (!Academy.IsInitialized)
            return;

        // 100 if this agent caught the ball, else 0. TB averages across agents/envs.
        Academy.Instance.StatsRecorder.Add(
            "GFSX/Grasp/CatchPercent",
            caughtBall ? 100f : 0f,
            StatAggregationMethod.Average);
    }
}
