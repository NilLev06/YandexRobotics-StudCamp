using System;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

/// <summary>
/// Receives compact detections produced by gfsx_real_vision.py.
/// The UDP listener is perception-only and never controls hardware.
/// </summary>
[DefaultExecutionOrder(-300)]
[DisallowMultipleComponent]
public sealed class GfsxRealVisionReceiver : MonoBehaviour
{
    [Serializable]
    private sealed class VisionPacket
    {
        public int schema;
        public long seq;
        public long sent_unix_ms;
        public int frame_width;
        public int frame_height;
        public float inference_ms;
        public bool sees;
        public float angle;
        public float distance = 1f;
        public float bbox_height_ratio;
        public float distance_metres;
        public float confidence;
        public float x1;
        public float y1;
        public float x2;
        public float y2;
    }

    private const int RequiredSchema = 1;

    [Header("Python detector on this Mac")]
    [SerializeField, Range(1024, 65535)] private int udpPort = 5005;
    [SerializeField, Range(0.1f, 2f)] private float staleAfterSeconds = 0.45f;
    [SerializeField, Range(0f, 1f)] private float minimumConfidence = 0.35f;

    private readonly ConcurrentQueue<string> pendingPackets =
        new ConcurrentQueue<string>();
    private Thread receiverThread;
    private UdpClient udp;
    private volatile bool receiverRunning;
    private string listenerError = string.Empty;
    private long lastSequence = -1;
    private float lastPacketTime = float.NegativeInfinity;
    private float lastDetectionTime = float.NegativeInfinity;
    private bool seesBall;
    private float normalizedAngle;
    private float normalizedDistance = 1f;
    private float bboxHeightRatio;
    private float distanceMetres;
    private float confidence;
    private float lastKnownDirection;
    private float inferenceMilliseconds;
    private int frameWidth;
    private int frameHeight;
    private int validPacketCount;
    private int rejectedPacketCount;

    public int UdpPort => udpPort;
    public bool ListenerRunning => receiverRunning;
    public string ListenerError => listenerError;
    public int ValidPacketCount => validPacketCount;
    public int RejectedPacketCount => rejectedPacketCount;
    public bool HasFreshPacket =>
        validPacketCount > 0 &&
        Time.realtimeSinceStartup - lastPacketTime <= staleAfterSeconds;
    public bool BallVisible => HasFreshPacket && seesBall;
    public float NormalizedAngle => BallVisible ? normalizedAngle : 0f;
    public float NormalizedDistance => BallVisible ? normalizedDistance : 1f;
    public float BboxHeightRatio => BallVisible ? bboxHeightRatio : 0f;
    public float DistanceMetres => BallVisible ? distanceMetres : 0f;
    public float Confidence => BallVisible ? confidence : 0f;
    public float LastKnownDirection => lastKnownDirection;
    public float InferenceMilliseconds => inferenceMilliseconds;
    public int FrameWidth => frameWidth;
    public int FrameHeight => frameHeight;
    public float PacketAgeSeconds => validPacketCount == 0
        ? float.PositiveInfinity
        : Mathf.Max(0f, Time.realtimeSinceStartup - lastPacketTime);
    public float SecondsSinceLastDetection =>
        float.IsNegativeInfinity(lastDetectionTime)
            ? 0f
            : Mathf.Max(0f, Time.realtimeSinceStartup - lastDetectionTime);

    private void OnEnable()
    {
        StartReceiver();
    }

    private void Update()
    {
        // Keep only the newest datagram when inference briefly outruns Unity.
        string newest = null;
        while (pendingPackets.TryDequeue(out string candidate))
            newest = candidate;
        if (newest != null)
            ApplyPacket(newest);

        if (!HasFreshPacket)
        {
            seesBall = false;
            normalizedAngle = 0f;
            normalizedDistance = 1f;
            confidence = 0f;
        }
    }

    private void OnDisable()
    {
        StopReceiver();
    }

    private void OnDestroy()
    {
        StopReceiver();
    }

    private void OnValidate()
    {
        udpPort = Mathf.Clamp(udpPort, 1024, 65535);
        staleAfterSeconds = Mathf.Clamp(staleAfterSeconds, 0.1f, 2f);
        minimumConfidence = Mathf.Clamp01(minimumConfidence);
    }

    public void Configure(int port, float staleSeconds)
    {
        udpPort = Mathf.Clamp(port, 1024, 65535);
        staleAfterSeconds = Mathf.Clamp(staleSeconds, 0.1f, 2f);
    }

    public void ResetDetectionMemory()
    {
        seesBall = false;
        normalizedAngle = 0f;
        normalizedDistance = 1f;
        bboxHeightRatio = 0f;
        distanceMetres = 0f;
        confidence = 0f;
        lastKnownDirection = 0f;
        lastDetectionTime = float.NegativeInfinity;
    }

    private void StartReceiver()
    {
        StopReceiver();
        listenerError = string.Empty;
        try
        {
            udp = new UdpClient(new IPEndPoint(IPAddress.Loopback, udpPort));
            udp.Client.ReceiveTimeout = 250;
            receiverRunning = true;
            receiverThread = new Thread(ReceiveLoop)
            {
                IsBackground = true,
                Name = "GFS-X real-vision UDP receiver"
            };
            receiverThread.Start();
        }
        catch (Exception exception)
        {
            listenerError = exception.Message;
            receiverRunning = false;
            udp?.Close();
            udp = null;
        }
    }

    private void StopReceiver()
    {
        receiverRunning = false;
        try
        {
            udp?.Close();
        }
        catch (Exception)
        {
            // Closing an already-disposed socket is harmless here.
        }
        udp = null;
        if (receiverThread != null && receiverThread.IsAlive)
            receiverThread.Join(400);
        receiverThread = null;
    }

    private void ReceiveLoop()
    {
        while (receiverRunning)
        {
            try
            {
                IPEndPoint sender = new IPEndPoint(IPAddress.Any, 0);
                byte[] bytes = udp.Receive(ref sender);
                if (!IPAddress.IsLoopback(sender.Address))
                    continue;
                if (bytes.Length == 0 || bytes.Length > 8192)
                    continue;
                pendingPackets.Enqueue(Encoding.UTF8.GetString(bytes));
            }
            catch (SocketException exception)
            {
                if (exception.SocketErrorCode != SocketError.TimedOut &&
                    receiverRunning)
                {
                    listenerError = exception.Message;
                }
            }
            catch (ObjectDisposedException)
            {
                break;
            }
            catch (Exception exception)
            {
                if (receiverRunning)
                    listenerError = exception.Message;
            }
        }
    }

    private void ApplyPacket(string json)
    {
        VisionPacket packet;
        try
        {
            packet = JsonUtility.FromJson<VisionPacket>(json);
        }
        catch (Exception)
        {
            rejectedPacketCount++;
            return;
        }

        if (packet == null || packet.schema != RequiredSchema ||
            packet.seq <= lastSequence || packet.frame_width <= 0 ||
            packet.frame_height <= 0 ||
            !Finite(packet.angle) || !Finite(packet.distance) ||
            !Finite(packet.bbox_height_ratio) ||
            !Finite(packet.distance_metres) || !Finite(packet.confidence) ||
            !Finite(packet.inference_ms))
        {
            rejectedPacketCount++;
            return;
        }

        lastSequence = packet.seq;
        lastPacketTime = Time.realtimeSinceStartup;
        validPacketCount++;
        frameWidth = packet.frame_width;
        frameHeight = packet.frame_height;
        inferenceMilliseconds = Mathf.Max(0f, packet.inference_ms);
        bboxHeightRatio = Mathf.Clamp01(packet.bbox_height_ratio);
        distanceMetres = Mathf.Max(0f, packet.distance_metres);
        confidence = Mathf.Clamp01(packet.confidence);
        seesBall = packet.sees && confidence >= minimumConfidence;
        normalizedAngle = seesBall
            ? Mathf.Clamp(packet.angle, -1f, 1f)
            : 0f;
        normalizedDistance = seesBall
            ? Mathf.Clamp01(packet.distance)
            : 1f;
        if (seesBall)
        {
            lastKnownDirection = normalizedAngle;
            lastDetectionTime = Time.realtimeSinceStartup;
        }
    }

    private static bool Finite(float value)
    {
        return !float.IsNaN(value) && !float.IsInfinity(value);
    }
}

