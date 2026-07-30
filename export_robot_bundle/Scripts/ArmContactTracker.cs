using UnityEngine;

/// <summary>
/// Tracks recent collisions between the arm / claw and external geometry.
/// </summary>
[DisallowMultipleComponent]
public sealed class ArmContactTracker : MonoBehaviour
{
    private const string ShoulderPivotName = "S1_Shoulder_Pivot";

    private int recentContacts;
    private int recentFloorContacts;
    private Transform armColliderRoot;

    public bool HasRecentContact => recentContacts > 0;
    public bool HasRecentFloorContact => recentFloorContacts > 0;

    private void Awake()
    {
        CacheArmColliderRoot();
        WireArmContactForwarders();
        ApplyGroundCollisionFilters();
    }

    public void ApplyGroundCollisionFilters()
    {
        if (armColliderRoot == null)
            CacheArmColliderRoot();

        Collider[] colliders = GetComponentsInChildren<Collider>(true);
        IgnoreClawGroundCollisions(colliders);
    }

    private void CacheArmColliderRoot()
    {
        armColliderRoot = transform.Find(ShoulderPivotName);
    }

    private void WireArmContactForwarders()
    {
        Collider[] colliders = GetComponentsInChildren<Collider>(true);
        for (int i = 0; i < colliders.Length; i++)
        {
            Collider collider = colliders[i];
            if (!IsArmCollider(collider))
                continue;

            ArmContactForwarder forwarder = collider.GetComponent<ArmContactForwarder>();
            if (forwarder == null)
                forwarder = collider.gameObject.AddComponent<ArmContactForwarder>();

            forwarder.Bind(this);
        }
    }

    /// <summary>
    /// Prevents the claw from acting as a jack that lifts the chassis off the floor.
    /// Ball collisions stay enabled so grasping still works.
    /// </summary>
    private void IgnoreClawGroundCollisions(Collider[] clawColliders)
    {
        Transform arenaRoot = GetComponentInParent<TrainingArena>() != null
            ? GetComponentInParent<TrainingArena>().transform
            : transform.root;

        Transform groundTransform = arenaRoot != null ? arenaRoot.Find("Ground") : null;
        if (groundTransform == null)
        {
            GameObject groundObject = GameObject.Find("Ground");
            if (groundObject != null)
                groundTransform = groundObject.transform;
        }

        if (groundTransform == null)
            return;

        Collider[] groundColliders = groundTransform.GetComponentsInChildren<Collider>(true);
        for (int clawIndex = 0; clawIndex < clawColliders.Length; clawIndex++)
        {
            Collider clawCollider = clawColliders[clawIndex];
            if (!IsArmCollider(clawCollider))
                continue;

            for (int groundIndex = 0; groundIndex < groundColliders.Length; groundIndex++)
            {
                Collider groundCollider = groundColliders[groundIndex];
                if (groundCollider == null || !groundCollider.enabled)
                    continue;

                Physics.IgnoreCollision(clawCollider, groundCollider, true);
            }
        }
    }

    internal void RegisterContact(Collider other)
    {
        if (IsIgnorable(other))
            return;

        recentContacts = Mathf.Max(recentContacts, 8);
        if (IsFloorLike(other))
            recentFloorContacts = Mathf.Max(recentFloorContacts, 8);
    }

    private void FixedUpdate()
    {
        if (recentContacts > 0)
            recentContacts--;
        if (recentFloorContacts > 0)
            recentFloorContacts--;
    }

    private bool IsArmCollider(Collider collider)
    {
        if (collider == null || !collider.enabled)
            return false;

        // Never ignore chassis colliders on the robot root — that makes the
        // whole rover fall through the floor.
        if (collider.transform == transform)
            return false;

        if (armColliderRoot == null)
            return false;

        Transform colliderTransform = collider.transform;
        return colliderTransform == armColliderRoot ||
               colliderTransform.IsChildOf(armColliderRoot);
    }

    private bool IsIgnorable(Collider other)
    {
        if (other == null)
            return true;

        if (other.CompareTag("TargetBall"))
            return true;

        Transform otherRoot = other.attachedRigidbody != null
            ? other.attachedRigidbody.transform
            : other.transform.root;
        return otherRoot == transform.root;
    }

    private static bool IsFloorLike(Collider other)
    {
        if (other == null)
            return false;

        string name = other.gameObject.name;
        return name == "Ground" ||
               name.StartsWith("Ground") ||
               other.transform.root.name == "Ground";
    }
}

[DisallowMultipleComponent]
internal sealed class ArmContactForwarder : MonoBehaviour
{
    private ArmContactTracker owner;

    public void Bind(ArmContactTracker tracker)
    {
        owner = tracker;
    }

    private void OnCollisionEnter(Collision collision)
    {
        owner?.RegisterContact(collision.collider);
    }

    private void OnCollisionStay(Collision collision)
    {
        owner?.RegisterContact(collision.collider);
    }
}
