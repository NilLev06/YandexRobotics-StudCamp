using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// One fixed-size training field with domain-randomized ball / obstacle layouts.
/// Obstacles are nominally 0.36×0.24×0.14 m (±size tolerance), ball Ø5 cm.
/// </summary>
[DisallowMultipleComponent]
public sealed class TrainingArena : MonoBehaviour
{
    [Header("Arena members")]
    [SerializeField] private RobotBrain agent;
    [SerializeField] private Transform targetBall;
    [SerializeField] private Rigidbody ballBody;
    [SerializeField] private SimulatedYoloCamera yoloCamera;

    [Header("Fixed playable area (local XZ, metres)")]
    [SerializeField] private Vector2 halfExtents = new Vector2(2.8f, 2.0f);
    [SerializeField] private float ballHeight = 0.025f;
    [SerializeField] private float robotHeight = 0f;

    [Header("Fixed scales")]
    [SerializeField] private Vector3 robotScale = new Vector3(0.7873901f, 0.7873901f, 0.7873901f);
    [Tooltip("Unity sphere diameter is 1 m, so 0.05 → Ø5 cm.")]
    [SerializeField] private Vector3 ballScale = new Vector3(0.05f, 0.05f, 0.05f);
    [SerializeField] private Color ballColor = new Color(1f, 0.45f, 0.05f, 1f);

    [Header("Robot spawn (pose only)")]
    [SerializeField] private bool randomizeRobotPose = true;
    [SerializeField] private float robotSpawnRadius = 0.7f;
    [SerializeField] private Vector2 robotYawRangeDegrees = new Vector2(-180f, 180f);
    [Tooltip("StudCamp relay: spawn robot near the start (negative local Z) short side.")]
    [SerializeField] private bool relayStartEndSpawn = true;
    [SerializeField] private float relayStartZ = -1.7f;
    [SerializeField] private float relayStartXJitter = 0.25f;

    [Header("Ball spawn (pose only)")]
    [SerializeField] private float minBallDistanceFromRobot = 2.2f;
    [SerializeField] private float maxBallDistanceFromRobot = 4.0f;
    [Tooltip("Minimum clearance from arena walls when placing the ball (metres).")]
    [SerializeField, Min(0.1f)] private float ballWallClearance = 0.45f;
    [Tooltip("StudCamp relay: place ball near the far short side (positive local Z).")]
    [SerializeField] private bool relayFarBallSpawn = true;
    [SerializeField] private float relayBallZ = 1.7f;
    [SerializeField] private float relayBallXJitter = 0.9f;
    [Tooltip("Reject ball poses that are already in the camera FOV at episode start.")]
    [SerializeField] private bool requireBallOutsideInitialFov = true;

    [Header("Return home (red cube)")]
    [SerializeField] private Transform homeMarker;
    [SerializeField] private Vector3 homeMarkerLocalScale = new Vector3(0.35f, 0.08f, 0.35f);
    [SerializeField] private Color homeMarkerColor = new Color(0.92f, 0.12f, 0.12f, 1f);

    [Header("Ball domain randomization")]
    [SerializeField] private bool randomizeBallPhysics = true;
    [Tooltip("Per guide P5: ball mass multiplier range (±100% from baseline).")]
    [SerializeField] private Vector2 ballMassMultiplierRange = new Vector2(0.5f, 2.0f);
    [Tooltip("Per guide P5: ball scale multiplier range (±20%).")]
    [SerializeField] private Vector2 ballScaleMultiplierRange = new Vector2(0.8f, 1.2f);

    [Header("Obstacles 0.36 x 0.24 x 0.14 m (± size tolerance)")]
    [SerializeField] private bool spawnObstacles = true;
    [SerializeField] private int minObstacles = 8;
    [SerializeField] private int maxObstacles = 8;
    [SerializeField] private Vector3 obstacleSize = new Vector3(0.36f, 0.14f, 0.24f);
    [Tooltip("Per-edge manufacturing tolerance as a fraction of nominal size, e.g. 0.05 → ±5%.")]
    [SerializeField, Range(0f, 0.25f)] private float obstacleSizeTolerance = 0.08f;
    [SerializeField] private float robotPassageWidth = 0.55f;
    [SerializeField] private float clearRadiusAroundRobot = 0.7f;
    [SerializeField] private float clearRadiusAroundBall = 0.45f;
    [SerializeField] private float corridorHalfWidth = 0.38f;
    [SerializeField] private int maxPlacementAttempts = 60;
    [SerializeField] private Material obstacleMaterial;

    private readonly List<GameObject> spawnedObstacles = new List<GameObject>();
    private readonly List<Vector3> obstacleFootprints = new List<Vector3>();
    private Transform obstaclesRoot;
    private Vector3 robotLocalStart;
    private Quaternion robotLocalStartRotation;
    private bool scalesCached;
    private float baselineBallMass = 0.05f;
    private Material runtimeBallMaterial;
    private Material runtimeObstacleMaterial;
    private Material runtimeHomeMaterial;

    public RobotBrain Agent => agent;
    public Transform TargetBall => targetBall;
    public Transform HomeMarker => homeMarker;
    public Vector2 HalfExtents => halfExtents;

    public Vector3 ReturnTargetWorld =>
        homeMarker != null ? homeMarker.position : ArenaToWorld(new Vector3(0f, 0f, relayStartZ));

    private void Awake()
    {
        AutoWire();
        CacheFixedScales();
        CacheRobotStart();
        EnsureObstaclesRoot();
        EnforceFixedScales();
        ApplyBallAppearance();
    }

    public void Configure(
        RobotBrain robotBrain,
        Transform ball,
        SimulatedYoloCamera camera,
        Vector2 playableHalfExtents)
    {
        agent = robotBrain;
        targetBall = ball;
        yoloCamera = camera;
        halfExtents = playableHalfExtents;
        if (ball != null)
            ballBody = ball.GetComponent<Rigidbody>();
        if (yoloCamera != null && targetBall != null)
            yoloCamera.targetBall = targetBall;
        CacheFixedScales();
        CacheRobotStart();
        EnforceFixedScales();
        ApplyBallAppearance();
    }

    public void ResetEpisodeLayout()
    {
        AutoWire();
        EnsureObstaclesRoot();
        EnsureHomeMarker();
        EnforceFixedScales();
        ApplyBallAppearance();

        ClearObstacles();
        PlaceRobot();
        PlaceHomeMarker();
        PlaceBallOutsideInitialFov();
        EnforceFixedScales();
        ApplyBallPhysicsRandomization();
        if (spawnObstacles)
            RebuildObstacles();
        Physics.SyncTransforms();
    }

    private void ApplyBallPhysicsRandomization()
    {
        if (!randomizeBallPhysics || targetBall == null)
            return;

        float scaleMultiplier = Random.Range(
            ballScaleMultiplierRange.x,
            ballScaleMultiplierRange.y);
        targetBall.localScale = ballScale * scaleMultiplier;

        if (ballBody != null)
        {
            float baselineMass = baselineBallMass > 0.0001f ? baselineBallMass : 0.05f;
            ballBody.mass = baselineMass * Random.Range(
                ballMassMultiplierRange.x,
                ballMassMultiplierRange.y);
            ballBody.WakeUp();
        }
    }

    public Vector3 ArenaToWorld(Vector3 localPosition)
    {
        return transform.TransformPoint(localPosition);
    }

    public Quaternion ArenaToWorld(Quaternion localRotation)
    {
        return transform.rotation * localRotation;
    }

    private void AutoWire()
    {
        if (agent == null)
            agent = GetComponentInChildren<RobotBrain>(true);
        if (yoloCamera == null && agent != null)
            yoloCamera = agent.GetComponentInChildren<SimulatedYoloCamera>(true);
        if (targetBall == null)
        {
            foreach (Transform child in GetComponentsInChildren<Transform>(true))
            {
                if (child.CompareTag("TargetBall"))
                {
                    targetBall = child;
                    break;
                }
            }
        }

        if (targetBall != null && ballBody == null)
            ballBody = targetBall.GetComponent<Rigidbody>();

        if (ballBody != null && baselineBallMass <= 0.0001f)
            baselineBallMass = Mathf.Max(0.01f, ballBody.mass);

        if (yoloCamera != null && targetBall != null && yoloCamera.targetBall != targetBall)
            yoloCamera.targetBall = targetBall;
    }

    private void CacheFixedScales()
    {
        if (scalesCached)
            return;

        if (agent != null)
            robotScale = agent.transform.localScale;
        ballScale = new Vector3(0.05f, 0.05f, 0.05f);
        scalesCached = true;
    }

    private void EnforceFixedScales()
    {
        if (agent != null)
            agent.transform.localScale = robotScale;
        if (targetBall != null)
            targetBall.localScale = ballScale;
    }

    private void ApplyBallAppearance()
    {
        if (targetBall == null)
            return;

        var renderer = targetBall.GetComponent<MeshRenderer>();
        if (renderer == null)
            return;

        if (runtimeBallMaterial == null)
        {
            runtimeBallMaterial = new Material(renderer.sharedMaterial != null
                ? renderer.sharedMaterial
                : new Material(Shader.Find("Universal Render Pipeline/Lit")));
            runtimeBallMaterial.name = "TargetBall_Orange_Runtime";
        }

        if (runtimeBallMaterial.HasProperty("_BaseColor"))
            runtimeBallMaterial.SetColor("_BaseColor", ballColor);
        if (runtimeBallMaterial.HasProperty("_Color"))
            runtimeBallMaterial.SetColor("_Color", ballColor);

        renderer.sharedMaterial = runtimeBallMaterial;
    }

    private void CacheRobotStart()
    {
        if (agent == null)
            return;

        robotLocalStart = transform.InverseTransformPoint(agent.transform.position);
        robotLocalStartRotation = Quaternion.Inverse(transform.rotation) * agent.transform.rotation;
        robotLocalStart.y = robotHeight;
    }

    private void EnsureObstaclesRoot()
    {
        if (obstaclesRoot != null)
            return;

        Transform existing = transform.Find("RandomObstacles");
        if (existing != null)
        {
            obstaclesRoot = existing;
            return;
        }

        GameObject root = new GameObject("RandomObstacles");
        root.transform.SetParent(transform, false);
        obstaclesRoot = root.transform;
    }

    private void EnsureHomeMarker()
    {
        if (homeMarker != null)
            return;

        Transform existing = transform.Find("RedHomeCube");
        if (existing != null)
        {
            homeMarker = existing;
            return;
        }

        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.name = "RedHomeCube";
        cube.transform.SetParent(transform, false);
        cube.transform.localScale = homeMarkerLocalScale;
        // Marker only — robot drives onto the zone; no physical blocking.
        Collider col = cube.GetComponent<Collider>();
        if (col != null)
            col.isTrigger = true;
        homeMarker = cube.transform;
        ApplyHomeMarkerVisual(cube);
    }

    private void ApplyHomeMarkerVisual(GameObject cube)
    {
        var renderer = cube.GetComponent<MeshRenderer>();
        if (renderer == null)
            return;

        if (runtimeHomeMaterial == null)
        {
            runtimeHomeMaterial = new Material(renderer.sharedMaterial != null
                ? renderer.sharedMaterial
                : new Material(Shader.Find("Universal Render Pipeline/Lit")));
            runtimeHomeMaterial.name = "RedHomeCube_Runtime";
        }

        if (runtimeHomeMaterial.HasProperty("_BaseColor"))
            runtimeHomeMaterial.SetColor("_BaseColor", homeMarkerColor);
        if (runtimeHomeMaterial.HasProperty("_Color"))
            runtimeHomeMaterial.SetColor("_Color", homeMarkerColor);
        renderer.sharedMaterial = runtimeHomeMaterial;
    }

    private void PlaceHomeMarker()
    {
        EnsureHomeMarker();
        if (homeMarker == null)
            return;

        Vector3 local = new Vector3(0f, homeMarkerLocalScale.y * 0.5f, relayStartZ);
        homeMarker.SetPositionAndRotation(
            ArenaToWorld(local),
            ArenaToWorld(Quaternion.identity));
        homeMarker.localScale = homeMarkerLocalScale;
        ApplyHomeMarkerVisual(homeMarker.gameObject);
    }

    private void PlaceRobot()
    {
        if (agent == null)
            return;

        Vector3 localPos = robotLocalStart;
        Quaternion localRot = robotLocalStartRotation;

        if (relayStartEndSpawn)
        {
            float x = Random.Range(-relayStartXJitter, relayStartXJitter);
            // Slightly in front of the red home cube, still on the start short side.
            localPos = new Vector3(x, robotHeight, relayStartZ + 0.35f);
            // Face along +Z toward the far ball edge (search direction).
            localRot = Quaternion.Euler(0f, 0f, 0f);
        }
        else if (randomizeRobotPose)
        {
            for (int attempt = 0; attempt < maxPlacementAttempts; attempt++)
            {
                Vector2 xz = Random.insideUnitCircle * robotSpawnRadius;
                localPos = new Vector3(xz.x, robotHeight, xz.y);
                if (IsInsidePlayable(localPos.x, localPos.z, 0.4f))
                    break;
            }

            float yaw = Random.Range(robotYawRangeDegrees.x, robotYawRangeDegrees.y);
            localRot = Quaternion.Euler(0f, yaw, 0f);
        }

        agent.TeleportToArenaPose(ArenaToWorld(localPos), ArenaToWorld(localRot));
        agent.transform.localScale = robotScale;
    }

    private void PlaceBallOutsideInitialFov()
    {
        if (targetBall == null)
            return;

        Vector3 robotWorld = agent != null ? agent.transform.position : transform.position;
        Vector3 chosen = ArenaToWorld(new Vector3(0.8f, ballHeight, relayBallZ));
        float ballRadius = GetBallHorizontalRadius();
        bool placed = false;

        for (int attempt = 0; attempt < maxPlacementAttempts; attempt++)
        {
            float x = Random.Range(-relayBallXJitter, relayBallXJitter);
            // Prefer ball near corners of the far edge so it is off the forward FOV.
            if (attempt < maxPlacementAttempts / 2)
                x = (Random.value < 0.5f ? -1f : 1f) * Random.Range(0.45f, relayBallXJitter);

            float z = relayBallZ + Random.Range(-0.1f, 0.1f);
            Vector3 local = new Vector3(x, ballHeight, z);
            if (!IsInsidePlayable(local.x, local.z, ballWallClearance + ballRadius))
                continue;

            Vector3 world = ArenaToWorld(local);
            float distance = HorizontalDistance(world, robotWorld);
            if (distance < minBallDistanceFromRobot)
                continue;

            ApplyBallWorldPose(world);

            if (yoloCamera != null)
                yoloCamera.targetBall = targetBall;

            Physics.SyncTransforms();

            if (requireBallOutsideInitialFov &&
                yoloCamera != null &&
                yoloCamera.IsTargetGeometricallyVisible(targetBall))
            {
                continue;
            }

            chosen = world;
            placed = true;
            break;
        }

        if (!placed)
        {
            // Fallback: far-corner pose, then yaw the robot away from the ball.
            float x = Random.value < 0.5f ? -relayBallXJitter : relayBallXJitter;
            chosen = ArenaToWorld(new Vector3(x, ballHeight, relayBallZ));
            ApplyBallWorldPose(chosen);
            if (yoloCamera != null)
                yoloCamera.targetBall = targetBall;
            Physics.SyncTransforms();

            if (requireBallOutsideInitialFov &&
                agent != null &&
                yoloCamera != null &&
                yoloCamera.IsTargetGeometricallyVisible(targetBall))
            {
                Vector3 away = robotWorld - chosen;
                away.y = 0f;
                if (away.sqrMagnitude > 0.001f)
                {
                    Quaternion yawAway = Quaternion.LookRotation(away.normalized, Vector3.up);
                    agent.TeleportToArenaPose(robotWorld, yawAway);
                    Physics.SyncTransforms();
                }
            }
        }

        targetBall.localScale = ballScale;
        if (yoloCamera != null)
            yoloCamera.targetBall = targetBall;
    }

    private void ApplyBallWorldPose(Vector3 world)
    {
        if (ballBody != null)
        {
            ballBody.isKinematic = false;
            ballBody.linearVelocity = Vector3.zero;
            ballBody.angularVelocity = Vector3.zero;
            ballBody.position = world;
            ballBody.rotation = Quaternion.identity;
            ballBody.WakeUp();
        }
        else if (targetBall != null)
        {
            targetBall.SetPositionAndRotation(world, Quaternion.identity);
        }
    }

    private void RebuildObstacles()
    {
        obstacleFootprints.Clear();
        int count = Mathf.Max(minObstacles, maxObstacles);
        // Fixed count when min==max; otherwise inclusive random.
        if (minObstacles != maxObstacles)
            count = Random.Range(minObstacles, maxObstacles + 1);

        for (int i = 0; i < count; i++)
        {
            if (!TryCreateObstacle($"Obstacle_{i}"))
                break;
        }
    }

    private bool TryCreateObstacle(string objectName)
    {
        Vector3 robotWorld = agent != null ? agent.transform.position : transform.position;
        Vector3 ballWorld = targetBall != null ? targetBall.position : transform.position;

        for (int attempt = 0; attempt < maxPlacementAttempts; attempt++)
        {
            // Pick which of the three edge lengths stands vertical (random face down).
            GetRandomBoxPose(out Vector3 scale, out float height, out float footprintRadius);

            float yaw = Random.Range(0f, 360f);
            Vector3 local = RandomPointOnFloor(height * 0.5f, footprintRadius + 0.15f);
            Vector3 world = ArenaToWorld(local);

            if (HorizontalDistance(world, robotWorld) < clearRadiusAroundRobot + footprintRadius)
                continue;
            if (HorizontalDistance(world, ballWorld) < clearRadiusAroundBall + footprintRadius)
                continue;
            if (BlocksRobotBallCorridor(world, footprintRadius, robotWorld, ballWorld))
                continue;
            if (IsTooCloseToObstacles(world, robotPassageWidth + footprintRadius))
                continue;

            GameObject box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = objectName;
            box.transform.SetParent(obstaclesRoot, false);
            box.transform.position = world;
            box.transform.rotation = ArenaToWorld(Quaternion.Euler(0f, yaw, 0f));
            box.transform.localScale = scale;
            ApplyObstacleVisual(box);

            // Keep obstacle static / stable on the floor.
            var body = box.GetComponent<Rigidbody>();
            if (body != null)
                Destroy(body);

            spawnedObstacles.Add(box);
            obstacleFootprints.Add(world);
            return true;
        }

        return false;
    }

    /// <summary>
    /// Builds a box with nominal edges 0.36 / 0.24 / 0.14 m plus independent
    /// per-edge tolerance, then picks which face rests on the floor.
    /// </summary>
    private void GetRandomBoxPose(out Vector3 scale, out float height, out float footprintRadius)
    {
        float a = JitterEdge(Mathf.Abs(obstacleSize.x));
        float b = JitterEdge(Mathf.Abs(obstacleSize.y));
        float c = JitterEdge(Mathf.Abs(obstacleSize.z));

        // Choose which edge is vertical, then randomly assign the other two to X/Z.
        int up = Random.Range(0, 3);
        float length;
        float width;
        switch (up)
        {
            case 0:
                height = a;
                length = b;
                width = c;
                break;
            case 1:
                height = b;
                length = a;
                width = c;
                break;
            default:
                height = c;
                length = a;
                width = b;
                break;
        }

        if (Random.value < 0.5f)
        {
            float tmp = length;
            length = width;
            width = tmp;
        }

        scale = new Vector3(length, height, width);
        footprintRadius = 0.5f * Mathf.Max(length, width);
    }

    private float JitterEdge(float nominal)
    {
        if (obstacleSizeTolerance <= 0f || nominal <= 0f)
            return nominal;

        float factor = 1f + Random.Range(-obstacleSizeTolerance, obstacleSizeTolerance);
        return Mathf.Max(0.01f, nominal * factor);
    }

    private void ApplyObstacleVisual(GameObject obstacle)
    {
        var renderer = obstacle.GetComponent<MeshRenderer>();
        if (renderer == null)
            return;

        if (obstacleMaterial != null)
        {
            renderer.sharedMaterial = obstacleMaterial;
            return;
        }

        if (runtimeObstacleMaterial == null)
        {
            runtimeObstacleMaterial = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            runtimeObstacleMaterial.color = new Color(0.55f, 0.5f, 0.45f);
            if (runtimeObstacleMaterial.HasProperty("_BaseColor"))
                runtimeObstacleMaterial.SetColor("_BaseColor", new Color(0.55f, 0.5f, 0.45f));
        }

        renderer.sharedMaterial = runtimeObstacleMaterial;
    }

    private bool BlocksRobotBallCorridor(
        Vector3 worldPoint,
        float footprintRadius,
        Vector3 robotWorld,
        Vector3 ballWorld)
    {
        Vector3 a = new Vector3(robotWorld.x, 0f, robotWorld.z);
        Vector3 b = new Vector3(ballWorld.x, 0f, ballWorld.z);
        Vector3 p = new Vector3(worldPoint.x, 0f, worldPoint.z);
        return DistancePointToSegment(p, a, b) < corridorHalfWidth + footprintRadius;
    }

    private static float DistancePointToSegment(Vector3 point, Vector3 a, Vector3 b)
    {
        Vector3 ab = b - a;
        float lengthSq = ab.sqrMagnitude;
        if (lengthSq < 0.0001f)
            return Vector3.Distance(point, a);

        float t = Mathf.Clamp01(Vector3.Dot(point - a, ab) / lengthSq);
        return Vector3.Distance(point, a + ab * t);
    }

    private void ClearObstacles()
    {
        for (int i = 0; i < spawnedObstacles.Count; i++)
        {
            if (spawnedObstacles[i] != null)
                Destroy(spawnedObstacles[i]);
        }

        spawnedObstacles.Clear();
        obstacleFootprints.Clear();

        if (obstaclesRoot == null)
            return;

        for (int i = obstaclesRoot.childCount - 1; i >= 0; i--)
            Destroy(obstaclesRoot.GetChild(i).gameObject);
    }

    private Vector3 RandomBallPointOnFloor()
    {
        float inset = ballWallClearance + GetBallHorizontalRadius();
        return RandomPointOnFloor(ballHeight, inset);
    }

    private float GetBallHorizontalRadius()
    {
        return Mathf.Max(0.01f, ballScale.x * 0.5f);
    }

    private Vector3 RandomPointOnFloor(float y, float edgeInset)
    {
        float maxX = Mathf.Max(0.1f, halfExtents.x - edgeInset);
        float maxZ = Mathf.Max(0.1f, halfExtents.y - edgeInset);
        return new Vector3(
            Random.Range(-maxX, maxX),
            y,
            Random.Range(-maxZ, maxZ));
    }

    private bool IsInsidePlayable(float x, float z, float inset)
    {
        return Mathf.Abs(x) <= halfExtents.x - inset &&
               Mathf.Abs(z) <= halfExtents.y - inset;
    }

    private bool IsTooCloseToObstacles(Vector3 worldPoint, float minGap)
    {
        float minGapSq = minGap * minGap;
        for (int i = 0; i < obstacleFootprints.Count; i++)
        {
            if (HorizontalDistanceSq(worldPoint, obstacleFootprints[i]) < minGapSq)
                return true;
        }

        return false;
    }

    private static float HorizontalDistance(Vector3 a, Vector3 b)
    {
        float dx = a.x - b.x;
        float dz = a.z - b.z;
        return Mathf.Sqrt(dx * dx + dz * dz);
    }

    private static float HorizontalDistanceSq(Vector3 a, Vector3 b)
    {
        float dx = a.x - b.x;
        float dz = a.z - b.z;
        return dx * dx + dz * dz;
    }
}
