using System;
using UnityEngine;

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
    [SerializeField, Min(0.05f)] private float ultrasonicRange = 2f;
    [SerializeField, Range(1f, 90f)]
    private float ultrasonicConeAngleDegrees = 30f;
    [SerializeField, Range(1, 31)] private int ultrasonicRayCount = 7;

    [Header("Obstacle IR sensors")]
    [SerializeField, Min(0.01f)] private float obstacleIRRange = 0.15f;

    [Header("Gripper IR sensor")]
    [SerializeField, Min(0.01f)] private float gripperIRRange = 0.075f;
    [SerializeField, Range(0f, 0.03f)] private float gripperBeamRadius = 0.008f;
    [SerializeField] private string targetBallTag = "TargetBall";

    [Header("Physics filtering and debug")]
    [SerializeField] private LayerMask sensingLayers = ~0;
    [SerializeField] private bool drawDebugRays = true;

    private readonly RaycastHit[] hitBuffer =
        new RaycastHit[MaximumPhysicsHits];
    private readonly Collider[] overlapBuffer =
        new Collider[MaximumPhysicsHits];

    private float ultrasonicDistanceMetres;
    private float ultrasonicNormalized = 1f;
    private float leftIr;
    private float rightIr;
    private float gripperIr;
    private Collider detectedBallCollider;
    private Transform detectedBallTransform;

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
        SampleNow();
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
        leftIr = SampleObstacleIR(leftIRPoint) ? 1f : 0f;
        rightIr = SampleObstacleIR(rightIRPoint) ? 1f : 0f;
        gripperIr = SampleGripperIR() ? 1f : 0f;
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
    }

    private bool SampleObstacleIR(Transform sensorPoint)
    {
        if (sensorPoint == null)
            return true;

        bool hitSomething = TryRaycastNearest(
            sensorPoint.position,
            sensorPoint.forward,
            obstacleIRRange,
            requireTargetBall: false,
            ignoreTargetBall: true,
            out RaycastHit nearestHit,
            out _);

        DrawDebugBeam(
            sensorPoint.position,
            sensorPoint.forward,
            hitSomething ? nearestHit.distance : obstacleIRRange,
            hitSomething ? Color.red : Color.cyan);

        return hitSomething;
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
                    StringComparison.Ordinal))
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
}
