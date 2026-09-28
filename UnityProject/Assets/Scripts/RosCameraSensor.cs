using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;

/// <summary>
/// Publishes a pinhole RGB camera as sensor_msgs/CompressedImage (JPEG) plus CameraInfo.
/// The camera renders into its own RenderTexture; frames are read back asynchronously so the
/// simulation does not stall on the GPU. Place it on a child of base_link looking along +Z.
/// </summary>
[RequireComponent(typeof(Camera))]
public class RosCameraSensor : MonoBehaviour
{
    public string ImageTopic = "/camera/image_raw/compressed";
    public string InfoTopic = "/camera/camera_info";
    public string FrameId = "camera_rgb_optical_frame";
    public int Width = 640;
    public int Height = 480;
    // RealSense R200-like: ~42 deg vertical, ~54 deg horizontal at 4:3.
    public float VerticalFovDegrees = 42f;
    // Unity's default 0.3 m clips the floor at the bottom of the image (the camera is ~0.1 m up)
    // and any obstacle closer than that.
    public float NearClipPlane = 0.03f;
    public double PublishPeriodSeconds = 1.0 / 30.0;
    [Range(1, 100)]
    public int JpegQuality = 80;
    // Frames still being read back; skip new requests beyond this to avoid piling up latency.
    public int MaxReadbacksInFlight = 2;

    Camera m_Camera;
    RenderTexture m_Target;
    ROSConnection m_Ros;
    double m_TimeNextFrameSeconds;
    int m_ReadbacksInFlight;
    CameraInfoMsg m_Info;

    // What the camera renders (and publishes); null before Start.
    public RenderTexture Target => m_Target;

    void Start()
    {
        m_Camera = GetComponent<Camera>();
        m_Target = new RenderTexture(Width, Height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
        m_Camera.targetTexture = m_Target;
        m_Camera.fieldOfView = VerticalFovDegrees;
        m_Camera.nearClipPlane = NearClipPlane;
        m_Camera.depth = -10;  // never draw over the Game view camera

        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<CompressedImageMsg>(ImageTopic);
        m_Ros.RegisterPublisher<CameraInfoMsg>(InfoTopic);
        m_Info = MakeCameraInfo();
        m_TimeNextFrameSeconds = Clock.Now + PublishPeriodSeconds;
    }

    CameraInfoMsg MakeCameraInfo()
    {
        var fy = Height / 2.0 / Mathf.Tan(VerticalFovDegrees * Mathf.Deg2Rad / 2f);
        var fx = fy;  // square pixels
        var cx = Width / 2.0;
        var cy = Height / 2.0;
        return new CameraInfoMsg
        {
            width = (uint)Width,
            height = (uint)Height,
            distortion_model = "plumb_bob",
            d = new double[5],
            k = new[] { fx, 0, cx, 0, fy, cy, 0, 0, 1 },
            r = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
            p = new[] { fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0 },
        };
    }

    void Update()
    {
        if (Clock.NowTimeInSeconds < m_TimeNextFrameSeconds)
        {
            return;
        }
        m_TimeNextFrameSeconds += PublishPeriodSeconds;
        if (m_TimeNextFrameSeconds < Clock.NowTimeInSeconds)
        {
            // Fell behind (slow frame); resynchronise instead of bursting.
            m_TimeNextFrameSeconds = Clock.NowTimeInSeconds + PublishPeriodSeconds;
        }
        if (m_ReadbacksInFlight >= MaxReadbacksInFlight)
        {
            return;
        }

        // The texture holds the frame rendered at the start of this frame.
        var stamp = new TimeStamp(Clock.time);
        m_ReadbacksInFlight++;
        AsyncGPUReadback.Request(m_Target, 0, TextureFormat.RGB24, request => OnReadback(request, stamp));
    }

    void OnReadback(AsyncGPUReadbackRequest request, TimeStamp stamp)
    {
        m_ReadbacksInFlight--;
        if (request.hasError || m_Ros == null)
        {
            return;
        }
        // Unity textures are stored bottom row first, which the JPEG encoder expects.
        var jpeg = ImageConversion.EncodeNativeArrayToJPG(request.GetData<byte>(),
            GraphicsFormat.R8G8B8_SRGB, (uint)Width, (uint)Height, 0, JpegQuality);
        var header = new HeaderMsg
        {
            frame_id = FrameId,
            stamp = new TimeMsg { sec = stamp.Seconds, nanosec = stamp.NanoSeconds },
        };
        m_Ros.Publish(ImageTopic, new CompressedImageMsg(header, "jpeg", jpeg.ToArray()));
        jpeg.Dispose();
        m_Info.header = header;
        m_Ros.Publish(InfoTopic, m_Info);
    }

    void OnDestroy()
    {
        if (m_Target != null)
        {
            m_Target.Release();
        }
    }
}
