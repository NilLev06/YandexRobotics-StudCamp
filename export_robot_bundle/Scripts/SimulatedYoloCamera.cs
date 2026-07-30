using UnityEngine;

[RequireComponent(typeof(Camera))]
public class SimulatedYoloCamera : MonoBehaviour
{
    [Header("Target Settings")]
    public Transform targetBall;

    [Header("Camera Settings")]
    public float maxDetectionDistance = 2.0f;
    public float horizontalFOV = 40.0f;
    [Tooltip("Vertical FOV used for visibility checks. 0 = derive from the Unity camera.")]
    public float verticalFOV = 0f;
    public LayerMask obstacleLayers;

    [Header("Domain randomization (imperfect vision)")]
    [SerializeField] private bool enableDomainRandomization = true;
    [SerializeField, Range(0f, 0.15f)] private float angleNoiseStd = 0.03f;
    [SerializeField, Range(0f, 0.15f)] private float distanceNoiseStd = 0.04f;
    [SerializeField, Range(0f, 0.2f)] private float missDetectionChance = 0.04f;
    [SerializeField, Range(0f, 8f)] private float fovJitterDegrees = 3f;
    [SerializeField, Range(0f, 0.3f)] private float rangeJitterFraction = 0.08f;

    [Header("Outputs (Read Only)")]
    [SerializeField] private bool isBallVisible;
    [SerializeField] private float relativeAngleX;
    [SerializeField] private float relativeAngleY;
    [SerializeField] private float normalizedDistance;

    private Camera robotCamera;
    private float episodeAngleBias;
    private float episodeDistanceBias;
    private float episodeFovScale = 1f;
    private float episodeRangeScale = 1f;
    private float episodeMissChance;

    public bool IsBallVisible => isBallVisible;
    public float RelativeAngleX => relativeAngleX;
    public float RelativeAngleY => relativeAngleY;
    public float NormalizedDistance => normalizedDistance;
    public string DebugLastRejectReason { get; private set; } = "none";

    void Awake()
    {
        robotCamera = GetComponent<Camera>();
        RandomizeEpisodeNoise();
    }

    void FixedUpdate()
    {
        RefreshDetection();
    }

    public void RandomizeEpisodeNoise()
    {
        if (!enableDomainRandomization)
        {
            episodeAngleBias = 0f;
            episodeDistanceBias = 0f;
            episodeFovScale = 1f;
            episodeRangeScale = 1f;
            episodeMissChance = 0f;
            return;
        }

        episodeAngleBias = SampleGaussian(0f, angleNoiseStd);
        episodeDistanceBias = SampleGaussian(0f, distanceNoiseStd);
        episodeFovScale = 1f + Random.Range(-fovJitterDegrees, fovJitterDegrees) / Mathf.Max(1f, horizontalFOV);
        episodeRangeScale = 1f + Random.Range(-rangeJitterFraction, rangeJitterFraction);
        episodeMissChance = missDetectionChance * Random.Range(0.5f, 1.5f);
    }

    public void RefreshDetection()
    {
        if (targetBall == null)
        {
            DebugLastRejectReason = "no_target";
            ResetDetection();
            return;
        }

        EvaluateBallVisibility();
    }

    private void EvaluateBallVisibility()
    {
        Vector3 directionToBall = targetBall.position - transform.position;
        float distance = directionToBall.magnitude;
        float maxRange = maxDetectionDistance * episodeRangeScale;

        if (distance > maxRange)
        {
            DebugLastRejectReason = $"too_far:{distance:F2}>{maxRange:F2}";
            ResetDetection();
            return;
        }

        Vector3 localDirection = transform.InverseTransformDirection(directionToBall);
        float horizontalDistance = new Vector2(localDirection.x, localDirection.z).magnitude;
        float angleToBallX = Mathf.Atan2(localDirection.x, localDirection.z) * Mathf.Rad2Deg;
        float angleToBallY = Mathf.Atan2(localDirection.y, horizontalDistance) * Mathf.Rad2Deg;
        float halfFovX = 0.5f * horizontalFOV * episodeFovScale;
        float halfFovY = 0.5f * ResolveVerticalFovDegrees() * episodeFovScale;

        if (Mathf.Abs(angleToBallX) > halfFovX || Mathf.Abs(angleToBallY) > halfFovY)
        {
            DebugLastRejectReason =
                $"fov:x={angleToBallX:F1}/{halfFovX:F1} y={angleToBallY:F1}/{halfFovY:F1}";
            ResetDetection();
            return;
        }

        if (Physics.Raycast(transform.position, directionToBall.normalized, out RaycastHit hit, distance, obstacleLayers))
        {
            if (hit.transform != targetBall && !hit.transform.IsChildOf(targetBall))
            {
                DebugLastRejectReason = $"occluded:{hit.collider.name}";
                ResetDetection();
                return;
            }
        }

        if (enableDomainRandomization && Random.value < episodeMissChance)
        {
            DebugLastRejectReason = "miss_roll";
            ResetDetection();
            return;
        }

        isBallVisible = true;
        DebugLastRejectReason = "visible";
        Vector3 viewportPoint = robotCamera.WorldToViewportPoint(targetBall.position);
        float cleanAngleX = (viewportPoint.x - 0.5f) * 2f;
        float cleanAngleY = (viewportPoint.y - 0.5f) * 2f;
        float cleanDistance = Mathf.Clamp01(distance / maxDetectionDistance);

        relativeAngleX = Mathf.Clamp(cleanAngleX + episodeAngleBias + SampleGaussian(0f, angleNoiseStd * 0.35f), -1.5f, 1.5f);
        relativeAngleY = Mathf.Clamp(cleanAngleY + SampleGaussian(0f, angleNoiseStd * 0.25f), -1.5f, 1.5f);
        normalizedDistance = Mathf.Clamp01(cleanDistance + episodeDistanceBias + SampleGaussian(0f, distanceNoiseStd * 0.35f));
    }

    private float ResolveVerticalFovDegrees()
    {
        if (verticalFOV > 1f)
            return verticalFOV;

        if (robotCamera == null)
            robotCamera = GetComponent<Camera>();

        return robotCamera != null ? robotCamera.fieldOfView : horizontalFOV * 0.75f;
    }

    private void ResetDetection()
    {
        isBallVisible = false;
        relativeAngleX = 0f;
        relativeAngleY = 0f;
        normalizedDistance = 1f;
    }

    private static float SampleGaussian(float mean, float stdDev)
    {
        if (stdDev <= 0.00001f)
            return mean;

        float u1 = Mathf.Clamp01(1f - Random.value);
        float u2 = Random.value;
        float randStdNormal = Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Sin(2f * Mathf.PI * u2);
        return mean + stdDev * randStdNormal;
    }

    private void OnDrawGizmosSelected()
    {
        if (robotCamera == null) robotCamera = GetComponent<Camera>();

        Gizmos.color = isBallVisible ? Color.green : Color.red;
        Gizmos.DrawRay(transform.position, transform.forward * maxDetectionDistance);

        if (targetBall != null)
            Gizmos.DrawLine(transform.position, targetBall.position);
    }
}
