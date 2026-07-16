using UnityEngine;

[RequireComponent(typeof(Camera))]
public class SimulatedYoloCamera : MonoBehaviour
{
    [Header("Target Settings")]
    public Transform targetBall;

    [Header("Camera Settings")]
    public float maxDetectionDistance = 2.0f;
    public float horizontalFOV = 40.0f;
    public LayerMask obstacleLayers;

    [Header("Outputs (Read Only)")]
    [SerializeField] private bool isBallVisible;
    [SerializeField] private float relativeAngleX;
    [SerializeField] private float normalizedDistance;

    private Camera robotCamera;

    public bool IsBallVisible => isBallVisible;
    public float RelativeAngleX => relativeAngleX;
    public float NormalizedDistance => normalizedDistance;

    void Awake()
    {
        robotCamera = GetComponent<Camera>();
    }

    void Update()
    {
        if (targetBall == null)
        {
            ResetDetection();
            return;
        }

        EvaluateBallVisibility();
    }

    private void EvaluateBallVisibility()
    {
        Vector3 directionToBall = targetBall.position - transform.position;
        float distance = directionToBall.magnitude;

        if (distance > maxDetectionDistance)
        {
            ResetDetection();
            return;
        }

        Vector3 localDirection = transform.InverseTransformDirection(directionToBall);
        float angleToBall = Mathf.Atan2(localDirection.x, localDirection.z) * Mathf.Rad2Deg;

        if (Mathf.Abs(angleToBall) > horizontalFOV / 2f)
        {
            ResetDetection();
            return;
        }

        if (Physics.Raycast(transform.position, directionToBall.normalized, out RaycastHit hit, distance, obstacleLayers))
        {
            if (hit.transform != targetBall)
            {
                ResetDetection();
                return;
            }
        }

        isBallVisible = true;
        Vector3 viewportPoint = robotCamera.WorldToViewportPoint(targetBall.position);
        relativeAngleX = (viewportPoint.x - 0.5f) * 2f;
        normalizedDistance = Mathf.Clamp01(distance / maxDetectionDistance);
    }

    private void ResetDetection()
    {
        isBallVisible = false;
        relativeAngleX = 0f;
        normalizedDistance = 1f;
    }

    private void OnDrawGizmosSelected()
    {
        if (robotCamera == null) robotCamera = GetComponent<Camera>();

        Gizmos.color = isBallVisible ? Color.green : Color.red;
        Gizmos.DrawRay(transform.position, transform.forward * maxDetectionDistance);

        if (targetBall != null)
        {
            Gizmos.DrawLine(transform.position, targetBall.position);
        }
    }
}
