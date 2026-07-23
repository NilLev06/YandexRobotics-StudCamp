using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Logical grasping for the GFS-X claw. Capture requires the ball to sit
/// inside the jaw volume AND the gripper IR to see it — no remote stick.
/// </summary>
[DefaultExecutionOrder(100)]
[DisallowMultipleComponent]
public sealed class GripperController : MonoBehaviour
{
    private const int MaximumGraspCandidates = 32;
    private const string TargetBallTag = "TargetBall";
    private const float BallRadiusMetres = 0.025f;

    public enum ReleaseVelocityMode
    {
        Carrier,
        Zero,
        Captured
    }

    [Header("References")]
    [SerializeField] private GfsxArmRigController armController;
    [SerializeField] private VirtualSensors sensors;
    [SerializeField] private Transform holdPoint;
    [SerializeField] private Rigidbody carrierRigidbody;

    [Header("S4 grip thresholds (normalized)")]
    [SerializeField, Range(0.05f, 1f)]
    private float grabClosureThreshold = 0.70f;
    [SerializeField, Range(0f, 0.95f)]
    private float releaseClosureThreshold = 0.25f;

    [Header("Physical grasp volume (world metres)")]
    [SerializeField, Range(0.005f, 0.08f)]
    [Tooltip(
        "Sphere radius around HoldPoint. Keep near ball radius so the ball " +
        "cannot stick through closed jaws from outside.")]
    private float graspRadius = 0.028f;
    [Tooltip("Half-size of the jaw pocket in HoldPoint local space (metres).")]
    [SerializeField] private Vector3 jawPocketHalfExtents = new Vector3(0.022f, 0.018f, 0.03f);
    [Tooltip("Grab only when gripper IR reports the ball between the jaws.")]
    [SerializeField] private bool requireGripperIrForGrab = true;
    [SerializeField] private LayerMask graspLayers = ~0;
    [SerializeField] private bool drawGraspVolume = true;

    [Header("Release motion")]
    [SerializeField] private ReleaseVelocityMode releaseVelocity =
        ReleaseVelocityMode.Carrier;

    private Rigidbody heldBody;
    private Transform heldTransform;
    private GameObject heldTaggedBall;
    private Collider[] heldColliders = Array.Empty<Collider>();
    private bool[] heldColliderEnabledStates = Array.Empty<bool>();

    private Transform originalParent;
    private int originalSiblingIndex;
    private Vector3 originalLocalScale;
    private bool originalIsKinematic;
    private bool originalUseGravity;
    private bool originalDetectCollisions;
    private RigidbodyInterpolation originalInterpolation;
    private CollisionDetectionMode originalCollisionDetectionMode;
    private RigidbodyConstraints originalConstraints;
    private float originalLinearDamping;
    private float originalAngularDamping;
    private Vector3 capturedLinearVelocity;
    private Vector3 capturedAngularVelocity;

    private bool grabArmed;
    private float previousClosure01 = -1f;
    private readonly Collider[] graspCandidateBuffer =
        new Collider[MaximumGraspCandidates];

    public event Action<GameObject> Grabbed;
    public event Action<GameObject> Released;

    public GfsxArmRigController ArmController => armController;
    public VirtualSensors Sensors => sensors;
    public Transform HoldPoint => holdPoint;
    public Rigidbody CarrierRigidbody => carrierRigidbody;
    public bool IsHolding => heldBody != null && heldTransform != null;
    public GameObject HeldObject => heldTransform != null
        ? heldTransform.gameObject
        : null;
    public GameObject HeldBall => heldTaggedBall;
    public Rigidbody HeldRigidbody => heldBody;
    public float GrabClosureThreshold => grabClosureThreshold;
    public float ReleaseClosureThreshold => releaseClosureThreshold;
    public float GraspRadius => graspRadius;
    public bool RequireGripperIrForGrab => requireGripperIrForGrab;
    public bool IsGrabArmed => grabArmed;
    public float NormalizedClosure => armController != null
        ? armController.S4NormalizedClosure
        : 0f;
    public bool AreJawsOpen => NormalizedClosure <= releaseClosureThreshold;
    public bool AreJawsClosedEnough => NormalizedClosure >= grabClosureThreshold;
    public bool ConfigurationIsComplete =>
        armController != null && sensors != null && holdPoint != null;

    private void Reset()
    {
        AutoAssignMissingReferences();
        ValidateSettings();
    }

    private void Awake()
    {
        AutoAssignMissingReferences();
        ValidateSettings();
        InitializeGripCycleState();
    }

    private void OnEnable()
    {
        if (!Application.isPlaying)
            return;

        AutoAssignMissingReferences();
        InitializeGripCycleState();
    }

    private void OnValidate()
    {
        AutoAssignMissingReferences();
        ValidateSettings();
    }

    private void LateUpdate()
    {
        if (Application.isPlaying)
            EvaluateNow();
    }

    /// <summary>
    /// Capture only on an open→close edge: jaws must first go below the
    /// release threshold (open), then past the grab threshold (close).
    /// A claw that stays closed cannot vacuum-stick a ball that later
    /// enters the pocket.
    /// </summary>
    public void EvaluateNow()
    {
        AutoAssignMissingReferences();

        if (heldTransform == null)
        {
            if (!ReferenceEquals(heldBody, null) ||
                !ReferenceEquals(heldTransform, null))
                ClearHeldState();
        }
        else if (heldBody == null)
        {
            ReleaseAfterLostRigidbody();
        }

        float closure01 = NormalizedClosure;

        if (IsHolding)
        {
            if (holdPoint == null ||
                closure01 <= releaseClosureThreshold)
            {
                Release();
                // Opening after release re-arms for the next object.
                grabArmed = closure01 <= releaseClosureThreshold;
            }
            else
            {
                SnapHeldObjectToHoldPoint();
            }

            previousClosure01 = closure01;
            return;
        }

        if (!ConfigurationIsComplete)
        {
            previousClosure01 = closure01;
            return;
        }

        bool jawsOpen = closure01 <= releaseClosureThreshold;
        bool jawsClosed = closure01 >= grabClosureThreshold;

        if (jawsOpen)
        {
            grabArmed = true;
            previousClosure01 = closure01;
            return;
        }

        if (jawsClosed)
        {
            if (grabArmed)
            {
                // Rising-edge close after a real open. Always consume the arm
                // so a failed close cannot stick later without reopening.
                TryGrabDetectedBall(requireOpenCloseCycle: false);
                grabArmed = false;
            }

            previousClosure01 = closure01;
            return;
        }

        // Mid-travel: keep armed state only if we already saw a full open.
        previousClosure01 = closure01;
    }

    /// <summary>
    /// Grasps only when the ball is inside the jaw pocket, IR sees it, jaws
    /// are closed enough, and (by default) an open→close cycle was armed.
    /// </summary>
    public bool TryGrabDetectedBall(bool requireOpenCloseCycle = true)
    {
        if (IsHolding ||
            !ConfigurationIsComplete ||
            !AreJawsClosedEnough)
        {
            return false;
        }

        if (requireOpenCloseCycle && !grabArmed)
            return false;

        Physics.SyncTransforms();
        sensors.SampleNow();

        if (requireGripperIrForGrab && !sensors.GripperIrDetected)
            return false;

        if (!TryFindNearestBallInJawPocket(
                out Rigidbody volumeCandidate,
                out GameObject volumeTaggedBall))
        {
            return false;
        }

        bool grabbed = TryGrabInternal(volumeCandidate, volumeTaggedBall);
        if (grabbed)
            grabArmed = false;
        return grabbed;
    }

    /// <summary>
    /// Explicit ROS/ML entry point. Requires jaws closed enough, IR, jaw
    /// pocket, and an armed open→close cycle.
    /// </summary>
    public bool TryGrab(Rigidbody candidate)
    {
        if (candidate == null ||
            sensors == null ||
            !AreJawsClosedEnough ||
            !grabArmed)
        {
            return false;
        }

        Physics.SyncTransforms();
        sensors.SampleNow();

        if (requireGripperIrForGrab && !sensors.GripperIrDetected)
            return false;

        if (!TryFindNearestBallInJawPocket(
                out Rigidbody volumeCandidate,
                out GameObject volumeTaggedBall) ||
            volumeCandidate != candidate)
        {
            return false;
        }

        bool grabbed = TryGrabInternal(candidate, volumeTaggedBall);
        if (grabbed)
            grabArmed = false;
        return grabbed;
    }

    public bool Release()
    {
        if (heldTransform != null && heldBody == null)
            return ReleaseAfterLostRigidbody();

        if (!IsHolding)
        {
            ClearHeldState();
            return false;
        }

        Rigidbody bodyToRelease = heldBody;
        Transform transformToRelease = heldTransform;
        GameObject releasedBall = heldTaggedBall != null
            ? heldTaggedBall
            : transformToRelease.gameObject;

        Vector3 releasePosition = transformToRelease.position;
        Quaternion releaseRotation = transformToRelease.rotation;

        Transform parentToRestore = GetSafeOriginalParent(
            transformToRelease);
        transformToRelease.SetParent(parentToRestore, true);
        transformToRelease.SetPositionAndRotation(
            releasePosition,
            releaseRotation);
        transformToRelease.localScale = originalLocalScale;
        transformToRelease.SetSiblingIndex(
            Mathf.Max(0, originalSiblingIndex));

        bodyToRelease.interpolation = originalInterpolation;
        bodyToRelease.constraints = originalConstraints;
        bodyToRelease.linearDamping = originalLinearDamping;
        bodyToRelease.angularDamping = originalAngularDamping;
        bodyToRelease.isKinematic = originalIsKinematic;
        bodyToRelease.collisionDetectionMode =
            originalCollisionDetectionMode;
        bodyToRelease.useGravity = originalUseGravity;
        bodyToRelease.detectCollisions = originalDetectCollisions;

        if (!originalIsKinematic)
        {
            CalculateReleaseVelocity(
                releasePosition,
                out Vector3 linearVelocity,
                out Vector3 angularVelocity);
            bodyToRelease.linearVelocity = linearVelocity;
            bodyToRelease.angularVelocity = angularVelocity;
            bodyToRelease.WakeUp();
        }

        // Restore colliders last, after the object has been detached and all
        // Rigidbody settings have returned to their original values.
        int colliderCount = Mathf.Min(
            heldColliders.Length,
            heldColliderEnabledStates.Length);
        for (int index = 0; index < colliderCount; index++)
        {
            if (heldColliders[index] != null)
                heldColliders[index].enabled =
                    heldColliderEnabledStates[index];
        }

        Physics.SyncTransforms();
        ClearHeldState();
        Released?.Invoke(releasedBall);
        return true;
    }

    public void ForceRelease()
    {
        Release();
    }

    public void SetClosure01(float closure01)
    {
        AutoAssignMissingReferences();
        if (armController == null)
            return;

        float command = Mathf.Lerp(
            armController.S4MinimumClosureLimit,
            armController.S4MaximumClosureLimit,
            Mathf.Clamp01(closure01));
        armController.SetServoCommands(
            armController.S1Angle,
            armController.S2Angle,
            armController.S3Angle,
            command);
        EvaluateNow();
    }

    public void Open()
    {
        SetClosure01(0f);
    }

    public void Close()
    {
        SetClosure01(1f);
    }

    public void Configure(
        GfsxArmRigController arm,
        VirtualSensors virtualSensors,
        Transform gripHoldPoint,
        Rigidbody carrier)
    {
        armController = arm;
        sensors = virtualSensors;
        holdPoint = gripHoldPoint;
        carrierRigidbody = carrier;
    }

    [ContextMenu("Release Held Ball")]
    private void ReleaseFromContextMenu()
    {
        Release();
    }

    private bool TryGrabInternal(
        Rigidbody candidate,
        GameObject taggedBall)
    {
        if (IsHolding ||
            candidate == null ||
            taggedBall == null ||
            holdPoint == null ||
            NormalizedClosure < grabClosureThreshold)
        {
            return false;
        }

        if (!HasTargetBallTag(taggedBall))
            return false;

        Transform candidateTransform = candidate.transform;
        Transform taggedTransform = taggedBall.transform;
        if (taggedTransform != candidateTransform &&
            !taggedTransform.IsChildOf(candidateTransform))
        {
            // Moving only a Rigidbody child out of a tagged ancestor would
            // leave the logical TargetBall object behind.
            return false;
        }

        if (armController != null &&
            candidateTransform.IsChildOf(armController.transform))
        {
            return false;
        }

        Collider[] descendants =
            candidate.GetComponentsInChildren<Collider>(true);
        List<Collider> ownedColliders = new List<Collider>();
        for (int index = 0; index < descendants.Length; index++)
        {
            Collider collider = descendants[index];
            if (collider != null && collider.attachedRigidbody == candidate)
                ownedColliders.Add(collider);
        }

        if (ownedColliders.Count == 0)
            return false;

        heldBody = candidate;
        heldTransform = candidateTransform;
        heldTaggedBall = taggedBall;
        heldColliders = ownedColliders.ToArray();
        heldColliderEnabledStates = new bool[heldColliders.Length];
        for (int index = 0; index < heldColliders.Length; index++)
            heldColliderEnabledStates[index] = heldColliders[index].enabled;

        originalParent = candidateTransform.parent;
        originalSiblingIndex = candidateTransform.GetSiblingIndex();
        originalLocalScale = candidateTransform.localScale;
        originalIsKinematic = candidate.isKinematic;
        originalUseGravity = candidate.useGravity;
        originalDetectCollisions = candidate.detectCollisions;
        originalInterpolation = candidate.interpolation;
        originalCollisionDetectionMode = candidate.collisionDetectionMode;
        originalConstraints = candidate.constraints;
        originalLinearDamping = candidate.linearDamping;
        originalAngularDamping = candidate.angularDamping;
        capturedLinearVelocity = candidate.linearVelocity;
        capturedAngularVelocity = candidate.angularVelocity;

        if (!candidate.isKinematic)
        {
            candidate.linearVelocity = Vector3.zero;
            candidate.angularVelocity = Vector3.zero;
        }

        candidate.useGravity = false;
        candidate.detectCollisions = false;
        // Continuous collision modes are intended for dynamic bodies and can
        // warn or be downgraded when a Rigidbody becomes kinematic. Hold in
        // Discrete mode, then restore the exact original mode on release.
        candidate.collisionDetectionMode = CollisionDetectionMode.Discrete;
        candidate.isKinematic = true;

        for (int index = 0; index < heldColliders.Length; index++)
            heldColliders[index].enabled = false;

        // Keep world scale while crossing the imported FBX hierarchy, whose
        // claw ancestry has approximately 100x scale. SetParent(false) would
        // make a normal-sized ball roughly one hundred times too large.
        candidateTransform.SetParent(holdPoint, true);
        candidateTransform.SetPositionAndRotation(
            holdPoint.position,
            holdPoint.rotation);

        grabArmed = false;
        Grabbed?.Invoke(heldTaggedBall);
        return true;
    }

    private void SnapHeldObjectToHoldPoint()
    {
        if (heldTransform == null || holdPoint == null)
            return;

        // A kinematic Rigidbody under the imported, transform-driven FBX
        // hierarchy can lag a parent transform until the next physics sync.
        // Reassert the world pose in LateUpdate so the ball stays exactly in
        // the claw while S1/S2/S3 or the robot base moves.
        Vector3 targetPosition = holdPoint.position;
        Quaternion targetRotation = holdPoint.rotation;
        if (heldBody != null)
        {
            heldBody.position = targetPosition;
            heldBody.rotation = targetRotation;
        }

        heldTransform.SetPositionAndRotation(targetPosition, targetRotation);
    }

    private GameObject FindTaggedBallObject(Rigidbody candidate)
    {
        if (candidate == null)
            return null;

        Transform current = candidate.transform;
        while (current != null)
        {
            if (HasTargetBallTag(current))
                return current.gameObject;
            current = current.parent;
        }

        Transform[] descendants =
            candidate.GetComponentsInChildren<Transform>(true);
        for (int index = 0; index < descendants.Length; index++)
        {
            if (HasTargetBallTag(descendants[index]))
                return descendants[index].gameObject;
        }

        return null;
    }

    private bool TryFindNearestBallInJawPocket(
        out Rigidbody nearestBody,
        out GameObject nearestTaggedBall)
    {
        nearestBody = null;
        nearestTaggedBall = null;

        if (holdPoint == null || graspRadius <= 0f)
            return false;

        float searchRadius = Mathf.Max(
            graspRadius,
            jawPocketHalfExtents.magnitude + BallRadiusMetres * 0.5f);
        int hitCount = Physics.OverlapSphereNonAlloc(
            holdPoint.position,
            searchRadius,
            graspCandidateBuffer,
            graspLayers,
            QueryTriggerInteraction.Ignore);
        int count = Mathf.Min(hitCount, graspCandidateBuffer.Length);
        float nearestSquaredDistance = float.PositiveInfinity;

        for (int index = 0; index < count; index++)
        {
            Collider candidateCollider = graspCandidateBuffer[index];
            if (candidateCollider == null || !candidateCollider.enabled)
                continue;

            Rigidbody candidateBody =
                candidateCollider.attachedRigidbody ??
                candidateCollider.GetComponentInParent<Rigidbody>();
            if (candidateBody == null)
                continue;
            if (armController != null &&
                candidateBody.transform.IsChildOf(armController.transform))
            {
                continue;
            }

            GameObject taggedBall = FindTaggedBallObject(candidateBody);
            if (!HasTargetBallTag(taggedBall))
                continue;

            if (!IsBallCenterInsideJawPocket(taggedBall.transform.position))
                continue;

            float squaredDistance =
                (taggedBall.transform.position - holdPoint.position).sqrMagnitude;
            if (squaredDistance >= nearestSquaredDistance)
                continue;

            nearestSquaredDistance = squaredDistance;
            nearestBody = candidateBody;
            nearestTaggedBall = taggedBall;
        }

        return nearestBody != null && nearestTaggedBall != null;
    }

    /// <summary>
    /// Ball centre must sit inside the claw pocket (HoldPoint local box).
    /// Surface proximity alone is not enough — prevents stick-through.
    /// </summary>
    public bool IsBallCenterInsideJawPocket(Vector3 ballWorldPosition)
    {
        if (holdPoint == null)
            return false;

        Vector3 local = holdPoint.InverseTransformPoint(ballWorldPosition);
        Vector3 half = jawPocketHalfExtents;
        if (half.x < 0.005f) half.x = 0.005f;
        if (half.y < 0.005f) half.y = 0.005f;
        if (half.z < 0.005f) half.z = 0.005f;

        return Mathf.Abs(local.x) <= half.x &&
               Mathf.Abs(local.y) <= half.y &&
               Mathf.Abs(local.z) <= half.z;
    }

    private static bool HasTargetBallTag(GameObject candidate)
    {
        return candidate != null && string.Equals(
            candidate.tag,
            TargetBallTag,
            StringComparison.Ordinal);
    }

    private static bool HasTargetBallTag(Transform candidate)
    {
        return candidate != null && HasTargetBallTag(candidate.gameObject);
    }

    private void CalculateReleaseVelocity(
        Vector3 releasePosition,
        out Vector3 linearVelocity,
        out Vector3 angularVelocity)
    {
        switch (releaseVelocity)
        {
            case ReleaseVelocityMode.Captured:
                linearVelocity = capturedLinearVelocity;
                angularVelocity = capturedAngularVelocity;
                break;

            case ReleaseVelocityMode.Zero:
                linearVelocity = Vector3.zero;
                angularVelocity = Vector3.zero;
                break;

            default:
                if (carrierRigidbody != null)
                {
                    linearVelocity = carrierRigidbody.GetPointVelocity(
                        releasePosition);
                    angularVelocity = carrierRigidbody.angularVelocity;
                }
                else
                {
                    linearVelocity = Vector3.zero;
                    angularVelocity = Vector3.zero;
                }
                break;
        }
    }

    private bool ReleaseAfterLostRigidbody()
    {
        if (heldTransform == null)
        {
            ClearHeldState();
            return false;
        }

        Transform transformToRelease = heldTransform;
        GameObject releasedBall = heldTaggedBall != null
            ? heldTaggedBall
            : transformToRelease.gameObject;
        Vector3 releasePosition = transformToRelease.position;
        Quaternion releaseRotation = transformToRelease.rotation;

        transformToRelease.SetParent(
            GetSafeOriginalParent(transformToRelease),
            true);
        transformToRelease.SetPositionAndRotation(
            releasePosition,
            releaseRotation);
        transformToRelease.localScale = originalLocalScale;
        transformToRelease.SetSiblingIndex(
            Mathf.Max(0, originalSiblingIndex));

        int colliderCount = Mathf.Min(
            heldColliders.Length,
            heldColliderEnabledStates.Length);
        for (int index = 0; index < colliderCount; index++)
        {
            if (heldColliders[index] != null)
                heldColliders[index].enabled =
                    heldColliderEnabledStates[index];
        }

        Physics.SyncTransforms();
        ClearHeldState();
        Released?.Invoke(releasedBall);
        return true;
    }

    private Transform GetSafeOriginalParent(Transform objectToRelease)
    {
        if (originalParent == null ||
            objectToRelease == null ||
            originalParent.IsChildOf(objectToRelease))
        {
            return null;
        }

        return originalParent;
    }

    private void AutoAssignMissingReferences()
    {
        if (armController == null)
            armController = GetComponentInParent<GfsxArmRigController>();
        if (sensors == null && armController != null)
            sensors = armController.GetComponent<VirtualSensors>();
        if (holdPoint == null && armController != null)
            holdPoint = armController.HoldPoint;
        if (carrierRigidbody == null && armController != null)
            carrierRigidbody = armController.GetComponent<Rigidbody>();
    }

    private void ValidateSettings()
    {
        grabClosureThreshold = Mathf.Clamp(
            grabClosureThreshold,
            0.05f,
            1f);
        releaseClosureThreshold = Mathf.Clamp(
            releaseClosureThreshold,
            0f,
            Mathf.Max(0f, grabClosureThreshold - 0.05f));
        graspRadius = Mathf.Clamp(graspRadius, 0.005f, 0.08f);
        jawPocketHalfExtents = new Vector3(
            Mathf.Clamp(jawPocketHalfExtents.x, 0.005f, 0.06f),
            Mathf.Clamp(jawPocketHalfExtents.y, 0.005f, 0.06f),
            Mathf.Clamp(jawPocketHalfExtents.z, 0.005f, 0.08f));
    }

    private void InitializeGripCycleState()
    {
        previousClosure01 = NormalizedClosure;
        // Arm only when jaws start open. A closed claw at spawn must not grab
        // until the agent (or auto-close) opens then closes again.
        grabArmed = !IsHolding && AreJawsOpen;
    }

    private void ClearHeldState()
    {
        heldBody = null;
        heldTransform = null;
        heldTaggedBall = null;
        heldColliders = Array.Empty<Collider>();
        heldColliderEnabledStates = Array.Empty<bool>();
        originalParent = null;
        originalSiblingIndex = 0;
        originalLocalScale = Vector3.one;
    }

    private void OnDisable()
    {
        if (Application.isPlaying)
            Release();
    }

    private void OnDestroy()
    {
        if (Application.isPlaying)
            Release();
    }

    private void OnDrawGizmosSelected()
    {
        if (!drawGraspVolume || holdPoint == null)
            return;

        Gizmos.color = new Color(0.1f, 0.9f, 0.3f, 0.35f);
        Gizmos.matrix = holdPoint.localToWorldMatrix;
        Gizmos.DrawWireCube(Vector3.zero, jawPocketHalfExtents * 2f);
        Gizmos.matrix = Matrix4x4.identity;
        Gizmos.color = new Color(0.2f, 0.6f, 1f, 0.55f);
        Gizmos.DrawWireSphere(holdPoint.position, graspRadius);
    }
}
