using UnityEngine;

/// <summary>
/// Polar coverage disk around the camera. Sectors warm while observed and cool over time.
/// </summary>
[DisallowMultipleComponent]
[DefaultExecutionOrder(-250)]
public sealed class CameraCoverageMap : MonoBehaviour
{
    [Header("Map shape")]
    [SerializeField, Range(12, 72)] private int sectorCount = 36;
    [SerializeField, Min(0.5f)] private float coverageRadius = 3.5f;
    [SerializeField, Range(0.05f, 1f)] private float coolingPerSecond = 0.18f;
    [SerializeField, Range(0.1f, 2f)] private float observeFillPerSecond = 0.85f;

    [Header("References")]
    [SerializeField] private Transform coverageCenter;
    [SerializeField] private Transform bodyYawReference;
    [SerializeField] private Transform cameraTransform;
    [SerializeField] private SimulatedYoloCamera yoloCamera;

    [Header("Debug")]
    [SerializeField] private bool drawCoverageGizmos = true;

    private float[] sectorWarmth;

    public float MeanWarmth { get; private set; }
    public float CameraSectorColdness { get; private set; }
    public float LastRevealColdness { get; private set; }

    private void Awake()
    {
        EnsureBuffers();
        AutoWire();
    }

    private void FixedUpdate()
    {
        if (sectorWarmth == null || sectorWarmth.Length != sectorCount)
            EnsureBuffers();

        float deltaTime = Time.fixedDeltaTime;
        CoolSectors(deltaTime);

        if (!TryGetCameraLookYaw(out float lookYaw, out float halfFovDegrees))
        {
            CameraSectorColdness = 1f;
            LastRevealColdness = 0f;
            MeanWarmth = ComputeMeanWarmth();
            return;
        }

        LastRevealColdness = MarkObservedSectors(lookYaw, halfFovDegrees, deltaTime);
        CameraSectorColdness = LastRevealColdness;
        MeanWarmth = ComputeMeanWarmth();
    }

    public void ResetCoverage()
    {
        EnsureBuffers();
        for (int i = 0; i < sectorWarmth.Length; i++)
            sectorWarmth[i] = 0f;

        MeanWarmth = 0f;
        CameraSectorColdness = 1f;
        LastRevealColdness = 0f;
    }

    public void Configure(
        Transform camera,
        SimulatedYoloCamera cameraSensor,
        Transform bodyReference = null)
    {
        cameraTransform = camera;
        yoloCamera = cameraSensor;
        coverageCenter = camera != null ? camera : transform;
        if (bodyReference != null)
            bodyYawReference = bodyReference;
        AutoWire();
    }

    private void AutoWire()
    {
        if (cameraTransform == null && yoloCamera != null)
            cameraTransform = yoloCamera.transform;

        if (yoloCamera == null && cameraTransform != null)
            yoloCamera = cameraTransform.GetComponent<SimulatedYoloCamera>();

        if (coverageCenter == null)
            coverageCenter = cameraTransform != null ? cameraTransform : transform;

        if (bodyYawReference == null)
            bodyYawReference = transform;
    }

    private void EnsureBuffers()
    {
        sectorCount = Mathf.Max(12, sectorCount);
        if (sectorWarmth == null || sectorWarmth.Length != sectorCount)
            sectorWarmth = new float[sectorCount];
    }

    private void CoolSectors(float deltaTime)
    {
        float cooling = coolingPerSecond * deltaTime;
        for (int i = 0; i < sectorWarmth.Length; i++)
            sectorWarmth[i] = Mathf.Max(0f, sectorWarmth[i] - cooling);
    }

    private float MarkObservedSectors(float lookYawDegrees, float halfFovDegrees, float deltaTime)
    {
        float maxColdness = 0f;
        float fillAmount = observeFillPerSecond * deltaTime;
        float marginDegrees = Mathf.Max(4f, halfFovDegrees * 0.35f);

        for (int i = 0; i < sectorWarmth.Length; i++)
        {
            float sectorYaw = SectorIndexToWorldYaw(i);
            float delta = Mathf.Abs(Mathf.DeltaAngle(sectorYaw, lookYawDegrees));
            if (delta > halfFovDegrees + marginDegrees)
                continue;

            float edgeFactor = 1f - Mathf.Clamp01(delta / (halfFovDegrees + marginDegrees));
            float coldness = 1f - sectorWarmth[i];
            maxColdness = Mathf.Max(maxColdness, coldness);

            sectorWarmth[i] = Mathf.Clamp01(
                sectorWarmth[i] + fillAmount * Mathf.Lerp(0.35f, 1f, edgeFactor));
        }

        return maxColdness;
    }

    private bool TryGetCameraLookYaw(out float lookYawDegrees, out float halfFovDegrees)
    {
        lookYawDegrees = 0f;
        halfFovDegrees = 20f;

        Transform lookTransform = cameraTransform;
        if (lookTransform == null && yoloCamera != null)
            lookTransform = yoloCamera.transform;

        if (lookTransform == null)
            return false;

        Vector3 forward = lookTransform.forward;
        forward.y = 0f;
        if (forward.sqrMagnitude < 0.0001f)
            return false;

        lookYawDegrees = Mathf.Atan2(forward.x, forward.z) * Mathf.Rad2Deg;
        if (bodyYawReference != null)
        {
            Vector3 bodyForward = bodyYawReference.forward;
            bodyForward.y = 0f;
            if (bodyForward.sqrMagnitude > 0.0001f)
            {
                float bodyYaw = Mathf.Atan2(bodyForward.x, bodyForward.z) * Mathf.Rad2Deg;
                lookYawDegrees = Mathf.DeltaAngle(bodyYaw, lookYawDegrees);
            }
        }

        halfFovDegrees = yoloCamera != null
            ? yoloCamera.horizontalFOV * 0.5f
            : 20f;
        return true;
    }

    private float ComputeMeanWarmth()
    {
        if (sectorWarmth == null || sectorWarmth.Length == 0)
            return 0f;

        float sum = 0f;
        for (int i = 0; i < sectorWarmth.Length; i++)
            sum += sectorWarmth[i];

        return sum / sectorWarmth.Length;
    }

    private float SectorIndexToWorldYaw(int sectorIndex)
    {
        float sectorWidth = 360f / sectorCount;
        return sectorIndex * sectorWidth + sectorWidth * 0.5f - 180f;
    }

    private void OnDrawGizmosSelected()
    {
        if (!drawCoverageGizmos || sectorWarmth == null || sectorWarmth.Length == 0)
            return;

        AutoWire();
        Vector3 center = coverageCenter != null ? coverageCenter.position : transform.position;
        center.y = 0.02f;

        float sectorWidth = 360f / sectorCount;
        for (int i = 0; i < sectorWarmth.Length; i++)
        {
            float start = SectorIndexToWorldYaw(i) - sectorWidth * 0.5f;
            float end = start + sectorWidth;
            DrawSectorWedge(center, coverageRadius, start, end, sectorWarmth[i]);
        }

        if (TryGetCameraLookYaw(out float lookYaw, out float halfFov))
        {
            Gizmos.color = Color.cyan;
            Vector3 lookDir = Quaternion.Euler(0f, lookYaw, 0f) * Vector3.forward;
            Gizmos.DrawRay(center, lookDir * coverageRadius);
            DrawSectorWedge(center, coverageRadius, lookYaw - halfFov, lookYaw + halfFov, 1.1f);
        }
    }

    private static void DrawSectorWedge(
        Vector3 center,
        float radius,
        float startYaw,
        float endYaw,
        float warmth)
    {
        Gizmos.color = Color.Lerp(
            new Color(0.2f, 0.25f, 0.8f, 0.35f),
            new Color(0.2f, 0.9f, 0.3f, 0.55f),
            warmth);

        Vector3 previousPoint = center;
        const int segments = 8;
        float span = Mathf.DeltaAngle(startYaw, endYaw);
        for (int segment = 0; segment <= segments; segment++)
        {
            float t = segment / (float)segments;
            float yaw = startYaw + span * t;
            Vector3 point = center + Quaternion.Euler(0f, yaw, 0f) * Vector3.forward * radius;
            if (segment > 0)
            {
                Gizmos.DrawLine(center, point);
                Gizmos.DrawLine(previousPoint, point);
            }

            previousPoint = point;
        }
    }
}
