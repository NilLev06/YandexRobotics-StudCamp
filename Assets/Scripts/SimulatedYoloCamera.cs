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
    [SerializeField] private float normalizedDistance;

    private Camera robotCamera;
    private float episodeAngleBias;
    private float episodeDistanceBias;
    private float episodeFovScale = 1f;
    private float episodeRangeScale = 1f;
    private float episodeMissChance;

    public bool IsBallVisible => isBallVisible;
    public float RelativeAngleX => relativeAngleX;
    public float NormalizedDistance => normalizedDistance;

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
            ResetDetection();
            return;
        }

        Vector3 localDirection = transform.InverseTransformDirection(directionToBall);
        float angleToBall = Mathf.Atan2(localDirection.x, localDirection.z) * Mathf.Rad2Deg;
        float halfFov = 0.5f * horizontalFOV * episodeFovScale;

        if (Mathf.Abs(angleToBall) > halfFov)
        {
            ResetDetection();
            return;
        }

        if (Physics.Raycast(transform.position, directionToBall.normalized, out RaycastHit hit, distance, obstacleLayers))
        {
            if (hit.transform != targetBall && !hit.transform.IsChildOf(targetBall))
            {
                ResetDetection();
                return;
            }
        }

        if (enableDomainRandomization && Random.value < episodeMissChance)
        {
            ResetDetection();
            return;
        }

        isBallVisible = true;
        Vector3 viewportPoint = robotCamera.WorldToViewportPoint(targetBall.position);
        float cleanAngle = (viewportPoint.x - 0.5f) * 2f;
        float cleanDistance = Mathf.Clamp01(distance / maxDetectionDistance);

        relativeAngleX = Mathf.Clamp(cleanAngle + episodeAngleBias + SampleGaussian(0f, angleNoiseStd * 0.35f), -1.5f, 1.5f);
        normalizedDistance = Mathf.Clamp01(cleanDistance + episodeDistanceBias + SampleGaussian(0f, distanceNoiseStd * 0.35f));
    }

    private void ResetDetection()
    {
        isBallVisible = false;
        relativeAngleX = 0f;
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
