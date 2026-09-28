using RosMessageTypes.Vision;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

/// <summary>
/// Draws the camera detector's boxes (/detections_2d, pixels of the RosCameraSensor image) in the
/// Game view: full screen while FirstPersonDrive is driving, otherwise over a picture-in-picture of
/// the robot camera (toggle with PreviewKey). Put it on the robot root.
/// Boxes arrive one detector latency (~0.1 s) after the frame, so they trail fast motion slightly.
/// </summary>
public class DetectionOverlay : MonoBehaviour
{
    public string DetectionsTopic = "/detections_2d";
    public KeyCode PreviewKey = KeyCode.C;
    public bool ShowPreview = true;
    [Range(0.1f, 0.6f)]
    public float PreviewWidthFraction = 0.3f;
    // Boxes older than this (wall-clock seconds) are hidden, e.g. after the detector stops.
    public float MaxAgeSeconds = 0.5f;
    public int LineWidth = 2;

    // The ROS annotated image's palette (RGB). ROS picks by class index and this by a hash of the
    // class name, so a class may get a different colour here than in RViz.
    static readonly Color32[] k_Palette =
    {
        new Color32(255, 56, 56, 255), new Color32(255, 157, 151, 255), new Color32(255, 112, 31, 255),
        new Color32(255, 178, 29, 255), new Color32(207, 210, 49, 255), new Color32(72, 249, 10, 255),
        new Color32(146, 204, 23, 255), new Color32(61, 219, 134, 255), new Color32(0, 188, 211, 255),
        new Color32(20, 99, 209, 255),
    };

    RosCameraSensor m_Sensor;
    FirstPersonDrive m_Drive;
    Detection2DMsg[] m_Detections = new Detection2DMsg[0];
    float m_ReceivedAt = float.NegativeInfinity;
    GUIStyle m_LabelStyle;

    void Start()
    {
        m_Drive = GetComponent<FirstPersonDrive>();
        ROSConnection.GetOrCreateInstance().Subscribe<Detection2DArrayMsg>(DetectionsTopic, OnDetections);
    }

    void OnDetections(Detection2DArrayMsg msg)
    {
        m_Detections = msg.detections;
        m_ReceivedAt = Time.realtimeSinceStartup;
    }

    void Update()
    {
        if (Input.GetKeyDown(PreviewKey))
        {
            ShowPreview = !ShowPreview;
        }
    }

    void OnGUI()
    {
        if (m_Sensor == null)
        {
            // camera_link is added in the scene, so look it up lazily rather than once in Start.
            m_Sensor = GetComponentInChildren<RosCameraSensor>();
            if (m_Sensor == null)
            {
                return;
            }
        }

        Rect image;
        if (m_Drive != null && m_Drive.IsDriving)
        {
            // The first-person camera shares the sensor's pose and vertical FOV, so image pixels map
            // to the screen by one uniform scale about the centre. Wider screens see more at the
            // sides than the detector does; the faint frame marks what it sees.
            var scale = (float)Screen.height / m_Sensor.Height;
            var width = m_Sensor.Width * scale;
            image = new Rect((Screen.width - width) / 2, 0, width, Screen.height);
            DrawFrame(image, new Color(1, 1, 1, 0.25f), 1);
        }
        else if (ShowPreview && m_Sensor.Target != null)
        {
            var width = Screen.width * PreviewWidthFraction;
            var height = width * m_Sensor.Height / m_Sensor.Width;
            image = new Rect(Screen.width - width - 10, 10, width, height);
            GUI.DrawTexture(image, m_Sensor.Target, ScaleMode.StretchToFill, false);
            DrawFrame(image, Color.black, 1);
        }
        else
        {
            return;
        }

        if (m_LabelStyle == null)
        {
            m_LabelStyle = new GUIStyle(GUI.skin.label) { fontSize = 13, padding = new RectOffset(3, 3, 1, 1) };
            m_LabelStyle.normal.textColor = Color.white;
        }

        var fresh = Time.realtimeSinceStartup - m_ReceivedAt <= MaxAgeSeconds;
        if (!fresh)
        {
            var status = float.IsNegativeInfinity(m_ReceivedAt)
                ? "waiting for " + DetectionsTopic
                : "no detections received recently";
            DrawLabel(new Vector2(image.x, image.y), status, new Color(0, 0, 0, 0.6f), true);
            return;
        }

        var s = image.width / m_Sensor.Width;
        foreach (var d in m_Detections)
        {
            var b = d.bbox;
            var rect = new Rect(
                image.x + (float)(b.center.position.x - b.size_x / 2) * s,
                image.y + (float)(b.center.position.y - b.size_y / 2) * s,
                (float)b.size_x * s, (float)b.size_y * s);
            var name = d.results.Length > 0 ? d.results[0].hypothesis.class_id : "?";
            var score = d.results.Length > 0 ? d.results[0].hypothesis.score : 0;
            var colour = ColourFor(name);
            DrawFrame(rect, colour, LineWidth);
            DrawLabel(new Vector2(rect.x, rect.y), string.Format("{0} {1:0.00}", name, score), colour, false);
        }
    }

    static Color ColourFor(string name)
    {
        // Stable across runs (string.GetHashCode is not guaranteed to be).
        var hash = 0;
        foreach (var c in name)
        {
            hash = hash * 31 + c;
        }
        return k_Palette[(hash & int.MaxValue) % k_Palette.Length];
    }

    static void DrawFrame(Rect r, Color colour, int width)
    {
        var previous = GUI.color;
        GUI.color = colour;
        var t = Texture2D.whiteTexture;
        GUI.DrawTexture(new Rect(r.x, r.y, r.width, width), t);
        GUI.DrawTexture(new Rect(r.x, r.yMax - width, r.width, width), t);
        GUI.DrawTexture(new Rect(r.x, r.y, width, r.height), t);
        GUI.DrawTexture(new Rect(r.xMax - width, r.y, width, r.height), t);
        GUI.color = previous;
    }

    // Filled tag above (or, with below, under) the given top-left corner.
    void DrawLabel(Vector2 corner, string text, Color background, bool below)
    {
        var size = m_LabelStyle.CalcSize(new GUIContent(text));
        var top = below ? corner.y : Mathf.Max(0, corner.y - size.y);
        var rect = new Rect(corner.x, top, size.x, size.y);
        var previous = GUI.color;
        GUI.color = background;
        GUI.DrawTexture(rect, Texture2D.whiteTexture);
        GUI.color = previous;
        GUI.Label(rect, text, m_LabelStyle);
    }
}
