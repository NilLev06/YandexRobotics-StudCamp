using UnityEngine;
using Random = UnityEngine.Random;

/// <summary>
/// Virtual equivalents of the GFS-X ultrasonic and infrared sensors.
/// All sensor anchors use their local blue Z axis (Transform.forward) as
/// the beam direction.
/// </summary>
[DefaultExecutionOrder(-100)]
[DisallowMultipleComponent]
public sealed class VirtualSensors : MonoBehaviour
{
    private const int MaximumPhysicsHits = 64;

    [Header("Sensor anchors (blue Z axis = beam direction)")]
    [SerializeField] private Transform centerPoint;
    [SerializeField] private Transform leftIRPoint;
    [SerializeField] private Transform rightIRPoint;
    [SerializeField] private Transform gripperIRPoint;

    [Header("Ultrasonic sensor")]
    [Tooltip("Maximum simulated range: 3.43 metres (343 centimetres).")]
    [SerializeField, Min(0.05f)] private float ultrasonicRange = 3.43f;
    [SerializeField, Range(1f, 90f)]
    private float ultrasonicConeAngleDegrees = 30f;
    [SerializeField, Range(1, 31)] private int ultrasonicRayCount = 7;

    [Header("Obstacle IR sensors")]
    [Tooltip("Left/right obstacle sensor range: 0.15 metres (15 cm).")]
    [SerializeField, Min(0.01f)] private float obstacleIRRange = 0.15f;

    [Header("Gripper IR sensor")]
    [Tooltip("Short TargetBall detector range from the task guide: 7.5 cm.")]
    [SerializeField, Min(0.01f)] private float gripperIRRange = 0.075f;
    [SerializeField, Range(0f, 0.03f)] private float gripperBeamRadius = 0.008f;
    [SerializeField] private string targetBallTag = "TargetBall";

    [Header("Physics filtering and debug")]
    [SerializeField] private LayerMask sensingLayers = ~0;
    [SerializeField] private bool drawDebugRays = true;

    [Header("Arm occlusion")]
    [Tooltip("Arm hierarchy checked for beams blocked before external obstacles.")]
    [SerializeField] private Transform armOcclusionRoot;

    [Header("Domain randomization (imperfect sensors)")]
    [SerializeField] private bool enableDomainRandomization = true;
    [SerializeField, Range(0f, 0.08f)] private float ultrasonicNoiseStd = 0.015f;
    [SerializeField, Range(0f, 0.1f)] private float ultrasonicBiasRange = 0.02f;
    [SerializeField, Range(0f, 0.15f)] private float irFalsePositiveChance = 0.02f;
    [SerializeField, Range(0f, 0.15f)] private float irFalseNegativeChance = 0.03f;
    [SerializeField, Range(0f, 0.1f)] private float gripperIrFlipChance = 0.015f;

    private readonly RaycastHit[] hitBuffer =
        new RaycastHit[MaximumPhysicsHits];
    private readonly Collider[] overlapBuffer =
        new Collider[MaximumPhysicsHits];

    private float ultrasonicDistanceMetres;
    private float ultrasonicNormalized = 1f;
    private float leftIr;
    private float rightIr;
    private float gripperIr;
    private bool leftIrOccludedByArm;
    private bool rightIrOccludedByArm;
    private Collider detectedBallCollider;
    private Transform detectedBallTransform;

    private float episodeUltrasonicBias;
    private float episodeUltrasonicNoiseStd;
    private float episodeIrFalsePositive;
    private float episodeIrFalseNegative;
    private float episodeGripperFlip;

    public Transform CenterPoint => centerPoint;
    public Transform LeftIRPoint => leftIRPoint;
    public Transform RightIRPoint => rightIRPoint;
    public Transform GripperIRPoint => gripperIRPoint;

    public float UltrasonicRange => ultrasonicRange;
    public float UltrasonicConeAngleDegrees => ultrasonicConeAngleDegrees;
    public int UltrasonicRayCount => ultrasonicRayCount;
    public float ObstacleIRRange => obstacleIRRange;
    public float ObstacleIrRange => obstacleIRRange;
    public float GripperIRRange => gripperIRRange;
    public float GripperIrRange => gripperIRRange;

    public float UltrasonicDistanceMetres => ultrasonicDistanceMetres;
    public float UltrasonicDistanceMeters => ultrasonicDistanceMetres;
    public float UltrasonicNormalized => ultrasonicNormalized;
    public float LeftIr => leftIr;
    public float RightIr => rightIr;
    public float GripperIr => gripperIr;
    public bool LeftIrDetected => leftIr > 0.5f;
    public bool RightIrDetected => rightIr > 0.5f;
    public bool GripperIrDetected => gripperIr > 0.5f;
    public bool LeftIrOccludedByArm => leftIrOccludedByArm;
    public bool RightIrOccludedByArm => rightIrOccludedByArm;
    public Collider DetectedBallCollider => detectedBallCollider;
    public GameObject DetectedBall => detectedBallTransform != null
        ? detectedBallTransform.gameObject
        : null;
    public Rigidbody DetectedBallRigidbody => detectedBallCollider != null
        ? detectedBallCollider.attachedRigidbody ??
          detectedBallCollider.GetComponentInParent<Rigidbody>()
        : null;
    public bool ConfigurationIsComplete =>
        centerPoint != null &&
        leftIRPoint != null &&
        rightIRPoint != null &&
        gripperIRPoint != null;

    private void Reset()
    {
        AutoAssignMissingAnchors();
        ValidateSettings();
    }

    private void Awake()
    {
        AutoAssignMissingAnchors();
        RandomizeEpisodeNoise();
        SampleNow();
    }

    /// <summary>
    /// Draws a new per-episode noise profile for imperfect physical sensors.
    /// </summary>
    public void RandomizeEpisodeNoise()
    {
        if (!enableDomainRandomization)
        {
            episodeUltrasonicBias = 0f;
            episodeUltrasonicNoiseStd = 0f;
            episodeIrFalsePositive = 0f;
            episodeIrFalseNegative = 0f;
            episodeGripperFlip = 0f;
            return;
        }

        episodeUltrasonicBias = Random.Range(-ultrasonicBiasRange, ultrasonicBiasRange);
        episodeUltrasonicNoiseStd = ultrasonicNoiseStd * Random.Range(0.5f, 1.5f);
        episodeIrFalsePositive = irFalsePositiveChance * Random.Range(0.5f, 1.5f);
        episodeIrFalseNegative = irFalseNegativeChance * Random.Range(0.5f, 1.5f);
        episodeGripperFlip = gripperIrFlipChance * Random.Range(0.5f, 1.5f);
    }

    private void FixedUpdate()
    {
        SampleNow();
    }

    private void OnValidate()
    {
        AutoAssignMissingAnchors();
        ValidateSettings();
    }

    /// <summary>
    /// Samples every sensor immediately. ML-Agents, ROS code, and tests may
    /// call this after Physics.SyncTransforms() for a deterministic reading.
    /// </summary>
    public void SampleNow()
    {
        SampleUltrasonic();
        leftIr = CorruptBinaryIr(SampleObstacleIR(leftIRPoint, out leftIrOccludedByArm));
        rightIr = CorruptBinaryIr(SampleObstacleIR(rightIRPoint, out rightIrOccludedByArm));
        gripperIr = CorruptGripperIr(SampleGripperIR());
    }

    private float CorruptBinaryIr(bool detected)
    {
        if (!enableDomainRandomization)
            return detected ? 1f : 0f;

        if (detected && Random.value < episodeIrFalseNegative)
            return 0f;
        if (!detected && Random.value < episodeIrFalsePositive)
            return 1f;
        return detected ? 1f : 0f;
    }

    private float CorruptGripperIr(bool detected)
    {
        if (!enableDomainRandomization)
            return detected ? 1f : 0f;

        if (Random.value < episodeGripperFlip)
            return detected ? 0f : 1f;
        return detected ? 1f : 0f;
    }

    public void RefreshReadings()
    {
        SampleNow();
    }

    /// <summary>
    /// Used by the one-time scene setup tool. The references remain editable
    /// from the Inspector afterwards.
    /// </summary>
    public void ConfigureAnchors(
        Transform ultrasonic,
        Transform leftObstacleIR,
        Transform rightObstacleIR,
        Transform gripperBallIR)
    {
        centerPoint = ultrasonic;
        leftIRPoint = leftObstacleIR;
        rightIRPoint = rightObstacleIR;
        gripperIRPoint = gripperBallIR;
    }

    private void SampleUltrasonic()
    {
        ultrasonicDistanceMetres = ultrasonicRange;
        ultrasonicNormalized = 1f;

        if (centerPoint == null)
        {
            // A missing collision-avoidance sensor must fail safe rather
            // than telling the controller that the path is clear.
            ultrasonicDistanceMetres = 0f;
            ultrasonicNormalized = 0f;
            return;
        }

        Vector3 origin = centerPoint.position;
        int rayCount = Mathf.Max(1, ultrasonicRayCount);
        float halfCone = ultrasonicConeAngleDegrees * 0.5f;

        for (int index = 0; index < rayCount; index++)
        {
            float angle = rayCount == 1
                ? 0f
                : Mathf.Lerp(
                    -halfCone,
                    halfCone,
                    index / (float)(rayCount - 1));
            Vector3 direction = Quaternion.AngleAxis(
                angle,
                centerPoint.up) * centerPoint.forward;

            bool hitSomething = TryRaycastNearest(
                origin,
                direction,
                ultrasonicRange,
                requireTargetBall: false,
                ignoreTargetBall: true,
                out RaycastHit nearestHit,
                out _);

            float drawnDistance = ultrasonicRange;
            if (hitSomething)
            {
                ultrasonicDistanceMetres = Mathf.Min(
                    ultrasonicDistanceMetres,
                    nearestHit.distance);
                drawnDistance = nearestHit.distance;
            }

            DrawDebugBeam(
                origin,
                direction,
                drawnDistance,
                hitSomething ? Color.red : Color.green);
        }

        ultrasonicNormalized = Mathf.Clamp01(
            ultrasonicDistanceMetres / ultrasonicRange);

        if (enableDomainRandomization)
        {
            float noisyMetres = ultrasonicDistanceMetres +
                episodeUltrasonicBias +
                SampleGaussian(0f, episodeUltrasonicNoiseStd * ultrasonicRange);
            noisyMetres = Mathf.Clamp(noisyMetres, 0f, ultrasonicRange);
            ultrasonicDistanceMetres = noisyMetres;
            ultrasonicNormalized = Mathf.Clamp01(noisyMetres / ultrasonicRange);
        }
    }

    private bool SampleObstacleIR(Transform sensorPoint, out bool occludedByArm)
    {
        occludedByArm = false;

        if (sensorPoint == null)
            return true;

        Vector3 origin = sensorPoint.position;
        Vector3 direction = sensorPoint.forward;
        occludedByArm = IsBeamOccludedByArm(origin, direction, obstacleIRRange);

        bool hitSomething = TryRaycastNearest(
            origin,
            direction,
            obstacleIRRange,
            requireTargetBall: false,
            ignoreTargetBall: true,
            out RaycastHit nearestHit,
            out _);

        DrawDebugBeam(
            origin,
            direction,
            hitSomething ? nearestHit.distance : obstacleIRRange,
            occludedByArm ? Color.yellow : (hitSomething ? Color.red : Color.cyan));

        return hitSomething;
    }

    private bool IsBeamOccludedByArm(Vector3 origin, Vector3 direction, float range)
    {
        if (armOcclusionRoot == null)
            return false;

        int hitCount = Physics.RaycastNonAlloc(
            origin,
            direction,
            hitBuffer,
            range,
            sensingLayers,
            QueryTriggerInteraction.Ignore);

        float nearestArmDistance = float.PositiveInfinity;
        float nearestExternalDistance = float.PositiveInfinity;
        int count = Mathf.Min(hitCount, hitBuffer.Length);

        for (int index = 0; index < count; index++)
        {
            RaycastHit candidate = hitBuffer[index];
            Collider candidateCollider = candidate.collider;
            if (candidateCollider == null)
                continue;

            if (TryFindTaggedTargetBall(candidateCollider.transform, out _))
                continue;

            if (candidateCollider.transform.IsChildOf(armOcclusionRoot))
            {
                nearestArmDistance = Mathf.Min(nearestArmDistance, candidate.distance);
                continue;
            }

            if (candidateCollider.transform.IsChildOf(transform))
                continue;

            nearestExternalDistance = Mathf.Min(nearestExternalDistance, candidate.distance);
        }

        return nearestArmDistance < float.PositiveInfinity &&
               nearestArmDistance <= nearestExternalDistance;
    }

    private bool SampleGripperIR()
    {
        detectedBallCollider = null;
        detectedBallTransform = null;

        if (gripperIRPoint == null)
            return false;

        Vector3 origin = gripperIRPoint.position;
        Vector3 direction = gripperIRPoint.forward;

        // SphereCast does not report a collider that already encloses its
        // starting volume. This overlap fallback handles a ball touching
        // the sensor point between the jaws.
        int overlapCount = Physics.OverlapSphereNonAlloc(
            origin,
            gripperBeamRadius,
            overlapBuffer,
            sensingLayers,
            QueryTriggerInteraction.Collide);
        if (TrySelectNearestTargetBallOverlap(
                overlapCount,
                origin,
                out Collider overlappingBall,
                out Transform overlappingBallTransform))
        {
            detectedBallCollider = overlappingBall;
            detectedBallTransform = overlappingBallTransform;
            DrawDebugBeam(origin, direction, 0f, Color.magenta);
            return true;
        }

        int hitCount = Physics.SphereCastNonAlloc(
            origin,
            gripperBeamRadius,
            direction,
            hitBuffer,
            gripperIRRange,
            sensingLayers,
            QueryTriggerInteraction.Collide);

        bool hitBall = TrySelectNearestValidHit(
            hitCount,
            requireTargetBall: true,
            ignoreTargetBall: false,
            out RaycastHit nearestHit,
            out Transform taggedBall);

        if (hitBall)
        {
            detectedBallCollider = nearestHit.collider;
            detectedBallTransform = taggedBall;
        }

        DrawDebugBeam(
            origin,
            direction,
            hitBall ? nearestHit.distance : gripperIRRange,
            hitBall ? Color.magenta : Color.yellow);

        return hitBall;
    }

    private bool TrySelectNearestTargetBallOverlap(
        int overlapCount,
        Vector3 origin,
        out Collider nearestCollider,
        out Transform taggedBall)
    {
        nearestCollider = null;
        taggedBall = null;
        float nearestSquaredDistance = float.PositiveInfinity;
        int count = Mathf.Min(overlapCount, overlapBuffer.Length);

        for (int index = 0; index < count; index++)
        {
            Collider candidate = overlapBuffer[index];
            if (candidate == null ||
                !TryFindTaggedTargetBall(
                    candidate.transform,
                    out Transform candidateBall))
            {
                continue;
            }

            float squaredDistance =
                (candidate.ClosestPoint(origin) - origin).sqrMagnitude;
            if (squaredDistance >= nearestSquaredDistance)
                continue;

            nearestSquaredDistance = squaredDistance;
            nearestCollider = candidate;
            taggedBall = candidateBall;
        }

        return nearestCollider != null;
    }

    private bool TryRaycastNearest(
        Vector3 origin,
        Vector3 direction,
        float range,
        bool requireTargetBall,
        bool ignoreTargetBall,
        out RaycastHit nearestHit,
        out Transform taggedBall)
    {
        int hitCount = Physics.RaycastNonAlloc(
            origin,
            direction,
            hitBuffer,
            range,
            sensingLayers,
            QueryTriggerInteraction.Ignore);

        return TrySelectNearestValidHit(
            hitCount,
            requireTargetBall,
            ignoreTargetBall,
            out nearestHit,
            out taggedBall);
    }

    private bool TrySelectNearestValidHit(
        int hitCount,
        bool requireTargetBall,
        bool ignoreTargetBall,
        out RaycastHit nearestHit,
        out Transform taggedBall)
    {
        nearestHit = default;
        taggedBall = null;
        float nearestDistance = float.PositiveInfinity;
        int count = Mathf.Min(hitCount, hitBuffer.Length);

        for (int index = 0; index < count; index++)
        {
            RaycastHit candidate = hitBuffer[index];
            Collider candidateCollider = candidate.collider;
            if (candidateCollider == null)
                continue;

            bool isTargetBall = TryFindTaggedTargetBall(
                candidateCollider.transform,
                out Transform candidateBall);

            if (requireTargetBall && !isTargetBall)
                continue;
            if (ignoreTargetBall && isTargetBall)
                continue;

            // A grasped ball becomes a child of this robot in Step 6. Test
            // its tag before rejecting robot descendants so it remains
            // visible to the gripper sensor while held.
            if (!isTargetBall && candidateCollider.transform.IsChildOf(transform))
                continue;

            if (candidate.distance >= nearestDistance)
                continue;

            nearestDistance = candidate.distance;
            nearestHit = candidate;
            taggedBall = candidateBall;
        }

        return nearestHit.collider != null;
    }

    private bool TryFindTaggedTargetBall(
        Transform candidate,
        out Transform taggedBall)
    {
        Transform current = candidate;
        while (current != null)
        {
            if (string.Equals(
                    current.tag,
                    targetBallTag,
                    System.StringComparison.Ordinal))
            {
                taggedBall = current;
                return true;
            }

            current = current.parent;
        }

        taggedBall = null;
        return false;
    }

    private void AutoAssignMissingAnchors()
    {
        if (centerPoint == null)
            centerPoint = FindDescendant("CenterPoint");
        if (leftIRPoint == null)
            leftIRPoint = FindDescendant("LeftIRPoint");
        if (rightIRPoint == null)
            rightIRPoint = FindDescendant("RightIRPoint");
        if (gripperIRPoint == null)
            gripperIRPoint = FindDescendant("GripperIRPoint");
        if (armOcclusionRoot == null)
        {
            GfsxArmRigController armRig = GetComponentInChildren<GfsxArmRigController>(true);
            if (armRig != null)
                armOcclusionRoot = armRig.transform;
        }
    }

    private Transform FindDescendant(string objectName)
    {
        Transform[] descendants = GetComponentsInChildren<Transform>(true);
        for (int index = 0; index < descendants.Length; index++)
        {
            if (descendants[index].name == objectName)
                return descendants[index];
        }

        return null;
    }

    private void ValidateSettings()
    {
        ultrasonicRange = Mathf.Max(0.05f, ultrasonicRange);
        ultrasonicConeAngleDegrees = Mathf.Clamp(
            ultrasonicConeAngleDegrees,
            1f,
            90f);
        ultrasonicRayCount = Mathf.Clamp(ultrasonicRayCount, 1, 31);
        if (ultrasonicRayCount > 1 && ultrasonicRayCount % 2 == 0)
            ultrasonicRayCount = Mathf.Min(31, ultrasonicRayCount + 1);

        obstacleIRRange = Mathf.Max(0.01f, obstacleIRRange);
        gripperIRRange = Mathf.Max(0.01f, gripperIRRange);
        gripperBeamRadius = Mathf.Clamp(gripperBeamRadius, 0f, 0.03f);
        if (string.IsNullOrWhiteSpace(targetBallTag))
            targetBallTag = "TargetBall";
    }

    private void DrawDebugBeam(
        Vector3 origin,
        Vector3 direction,
        float distance,
        Color colour)
    {
        if (drawDebugRays)
            Debug.DrawRay(origin, direction * distance, colour, 0.05f);
    }

    private void OnDrawGizmosSelected()
    {
        if (centerPoint != null)
        {
            int rayCount = Mathf.Max(1, ultrasonicRayCount);
            float halfCone = ultrasonicConeAngleDegrees * 0.5f;
            Gizmos.color = new Color(0.2f, 1f, 0.3f, 0.9f);

            for (int index = 0; index < rayCount; index++)
            {
                float angle = rayCount == 1
                    ? 0f
                    : Mathf.Lerp(
                        -halfCone,
                        halfCone,
                        index / (float)(rayCount - 1));
                Vector3 direction = Quaternion.AngleAxis(
                    angle,
                    centerPoint.up) * centerPoint.forward;
                Gizmos.DrawLine(
                    centerPoint.position,
                    centerPoint.position + direction * ultrasonicRange);
            }
        }

        DrawGizmoRay(leftIRPoint, obstacleIRRange, Color.cyan);
        DrawGizmoRay(rightIRPoint, obstacleIRRange, Color.cyan);
        DrawGizmoRay(gripperIRPoint, gripperIRRange, Color.yellow);

        if (gripperIRPoint != null && gripperBeamRadius > 0f)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(
                gripperIRPoint.position,
                gripperBeamRadius);
            Gizmos.DrawWireSphere(
                gripperIRPoint.position +
                gripperIRPoint.forward * gripperIRRange,
                gripperBeamRadius);
        }
    }

    private static void DrawGizmoRay(
        Transform point,
        float distance,
        Color colour)
    {
        if (point == null)
            return;

        Gizmos.color = colour;
        Gizmos.DrawLine(
            point.position,
            point.position + point.forward * distance);
    }

    private static float SampleGaussian(float mean, float stdDev)
    {
        if (stdDev <= 0.00001f)
            return mean;

        float u1 = Mathf.Clamp01(1f - Random.value);
        float u2 = Random.value;
        float randStdNormal = Mathf.Sqrt(-2f * Mathf.Log(u1)) *
            Mathf.Sin(2f * Mathf.PI * u2);
        return mean + stdDev * randStdNormal;
    }
}
