using Unity.MLAgents;

/// <summary>
/// Grasp outcome counters for TensorBoard.
/// CatchPercent = 100 * catches / episodes.
/// </summary>
public static class TrainingGraspMetrics
{
    private static int totalEpisodes;
    private static int totalCatches;

    public static int TotalEpisodes => totalEpisodes;
    public static int TotalCatches => totalCatches;

    public static float SuccessRate =>
        totalEpisodes > 0 ? totalCatches / (float)totalEpisodes : 0f;

    /// <summary>Percentage of episodes where the ball was caught (0–100).</summary>
    public static float CatchPercent => SuccessRate * 100f;

    public static float WER =>
        totalEpisodes > 0 ? (totalEpisodes - totalCatches) / (float)totalEpisodes : 0f;

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

        StatsRecorder stats = Academy.Instance.StatsRecorder;
        // Per-episode 0/1 — TB Average ≈ rolling catch rate.
        stats.Add(
            "GFSX/Grasp/EpisodeSuccess",
            caughtBall ? 1f : 0f,
            StatAggregationMethod.Average);
        stats.Add("GFSX/Grasp/Catches", caughtBall ? 1f : 0f, StatAggregationMethod.Sum);
        // Explicit % of caught balls (cumulative over the run).
        stats.Add("GFSX/Grasp/CatchPercent", CatchPercent, StatAggregationMethod.MostRecent);
        stats.Add("GFSX/Grasp/SuccessRate", SuccessRate, StatAggregationMethod.MostRecent);
        stats.Add("GFSX/Grasp/WER", WER, StatAggregationMethod.MostRecent);
        stats.Add("GFSX/Grasp/CatchCountTotal", totalCatches, StatAggregationMethod.MostRecent);
        stats.Add("GFSX/Grasp/EpisodeCountTotal", totalEpisodes, StatAggregationMethod.MostRecent);
        stats.Add(
            "GFSX/Grasp/EpisodeMiss",
            caughtBall ? 0f : 1f,
            StatAggregationMethod.Average);
    }
}
