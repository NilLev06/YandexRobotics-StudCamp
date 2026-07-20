using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Clones one fixed-size TrainingArena into a grid (default 40 agents / fields).
/// </summary>
[DefaultExecutionOrder(-1000)]
[DisallowMultipleComponent]
public sealed class MultiArenaManager : MonoBehaviour
{
    [Header("Template")]
    [SerializeField] private TrainingArena templateArena;

    [Header("Grid (one Unity env)")]
    [SerializeField, Min(1)] private int arenaCount = 40;
    [SerializeField, Min(1)] private int columns = 8;
    [SerializeField] private Vector2 spacing = new Vector2(9f, 7f);
    [SerializeField] private bool cloneOnAwake = false;
    [SerializeField] private bool hideTemplateAfterClone = false;

    private readonly List<TrainingArena> arenas = new List<TrainingArena>();

    public IReadOnlyList<TrainingArena> Arenas => arenas;
    public int ArenaCount => arenas.Count;

    public void Configure(TrainingArena template, int count, int gridColumns, Vector2 arenaSpacing)
    {
        templateArena = template;
        arenaCount = Mathf.Max(1, count);
        columns = Mathf.Max(1, gridColumns);
        spacing = arenaSpacing;
        cloneOnAwake = false;
    }

    private void Awake()
    {
        if (cloneOnAwake)
            BuildArenas();
    }

    [ContextMenu("Build Arenas Now")]
    public void BuildArenas()
    {
        ClearClones();
        arenas.Clear();

        if (templateArena == null)
            templateArena = FindFirstObjectByType<TrainingArena>();

        if (templateArena == null)
        {
            Debug.LogError("MultiArenaManager: no TrainingArena template found.");
            return;
        }

        Vector3 gridOrigin = templateArena.transform.position;
        Quaternion rotation = templateArena.transform.rotation;

        templateArena.name = "Arena_0";
        templateArena.transform.SetPositionAndRotation(GridOffset(gridOrigin, 0), rotation);
        arenas.Add(templateArena);

        int clonesNeeded = Mathf.Max(0, arenaCount - 1);
        for (int i = 0; i < clonesNeeded; i++)
        {
            int index = i + 1;
            TrainingArena clone = Instantiate(
                templateArena,
                GridOffset(gridOrigin, index),
                rotation,
                transform);
            clone.name = $"Arena_{index}";
            arenas.Add(clone);
        }

        if (hideTemplateAfterClone && clonesNeeded > 0)
            templateArena.gameObject.SetActive(false);

        Debug.Log(
            $"MultiArenaManager: {arenas.Count} arenas " +
            $"({columns} x {(arenaCount + columns - 1) / columns}, " +
            $"spacing {spacing.x}x{spacing.y} m).");
    }

    private Vector3 GridOffset(Vector3 origin, int index)
    {
        int col = index % columns;
        int row = index / columns;
        return origin + new Vector3(col * spacing.x, 0f, row * spacing.y);
    }

    private void ClearClones()
    {
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            Transform child = transform.GetChild(i);
            if (child == null || string.IsNullOrEmpty(child.name))
                continue;
            if (!child.name.StartsWith("Arena_"))
                continue;
            if (templateArena != null && child == templateArena.transform)
                continue;

            // Immediate destroy so rebuild does not briefly register 2x agents.
            DestroyImmediate(child.gameObject);
        }
    }
}
