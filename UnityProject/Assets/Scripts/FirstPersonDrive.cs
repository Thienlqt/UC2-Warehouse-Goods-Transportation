using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosSharp.Control;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Press ToggleKey (P) to drive the robot from the keyboard, seen through its own camera.
/// Put it on the robot root next to AGVController. While driving, the Game view renders from the
/// RosCameraSensor's pose with the same vertical FOV, and the robot's meshes are hidden from that
/// view only (the ROS camera and physics are unaffected).
/// Keys: W/S or Up/Down drive, A/D or Left/Right turn. The Game view needs keyboard focus.
/// Assisted (default, AssistKey O toggles): the keys go to ROS on TeleopTopic and the robot
/// follows ROS cmd_vel. ROS teleop_assist holds the heading while W is held and steers around
/// obstacles ahead; cmd_vel_guard brakes. Direct: the keys drive the wheels, ROS is ignored.
/// </summary>
public class FirstPersonDrive : MonoBehaviour
{
    public KeyCode ToggleKey = KeyCode.P;
    // Where the view camera sits; defaults to the RosCameraSensor in the robot's children.
    public Transform ViewMount;
    // Used only when there is no RosCameraSensor to copy the FOV from.
    public float FallbackVerticalFovDegrees = 60f;
    public bool HideRobot = true;
    public bool ShowHelp = true;

    [Header("Assisted driving (ROS obstacle avoidance)")]
    public bool Assisted = true;
    public KeyCode AssistKey = KeyCode.O;
    public string TeleopTopic = "cmd_vel_teleop";
    public float LinearSpeed = 0.5f;     // m/s requested while W/S is held
    public float AngularSpeed = 0.5f;    // rad/s requested while A/D is held
    public float PublishPeriodSeconds = 0.05f;

    public bool IsDriving { get; private set; }

    AGVController m_Controller;
    ControlMode m_ModeBeforeDriving;
    Camera m_View;
    Renderer[] m_RobotRenderers = new Renderer[0];
    // Screen cameras, their FreeCam (also reads WASD/arrows) and audio listeners, off while driving.
    readonly List<Behaviour> m_Suspended = new List<Behaviour>();
    GUIStyle m_HelpStyle;
    ROSConnection m_Ros;
    float m_NextPublishTime;
    bool m_KeysHeld;

    void Start()
    {
        m_Controller = GetComponentInChildren<AGVController>();
        if (m_Controller == null)
        {
            Debug.LogWarning("FirstPersonDrive: no AGVController on this robot; driving disabled.");
            enabled = false;
            return;
        }
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<TwistMsg>(TeleopTopic);
    }

    void OnEnable()
    {
        RenderPipelineManager.beginCameraRendering += OnBeginCameraRendering;
        RenderPipelineManager.endCameraRendering += OnEndCameraRendering;
    }

    void OnDisable()
    {
        RenderPipelineManager.beginCameraRendering -= OnBeginCameraRendering;
        RenderPipelineManager.endCameraRendering -= OnEndCameraRendering;
        if (IsDriving)
        {
            StopDriving();
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(ToggleKey))
        {
            if (IsDriving)
            {
                StopDriving();
            }
            else
            {
                StartDriving();
            }
        }
        if (!IsDriving)
        {
            return;
        }
        if (Input.GetKeyDown(AssistKey))
        {
            if (Assisted)
            {
                PublishTeleop(0f, 0f);  // let go of ROS before driving directly
            }
            Assisted = !Assisted;
            m_Controller.mode = Assisted ? ControlMode.ROS : ControlMode.Keyboard;
        }
        if (Assisted && Time.time >= m_NextPublishTime)
        {
            m_NextPublishTime = Time.time + PublishPeriodSeconds;
            // ROS twist: +x forward, +z counter-clockwise (A / Left turns left).
            float forward = KeyDirection("Vertical");
            float left = -KeyDirection("Horizontal");
            m_KeysHeld = forward != 0 || left != 0;
            PublishTeleop(forward * LinearSpeed, left * AngularSpeed);
        }
    }

    static float KeyDirection(string axis)
    {
        float value = Input.GetAxisRaw(axis);
        return value > 0f ? 1f : value < 0f ? -1f : 0f;
    }

    void PublishTeleop(float linear, float angular)
    {
        if (m_Ros == null)
        {
            return;
        }
        var msg = new TwistMsg();
        msg.linear.x = linear;
        msg.angular.z = angular;
        m_Ros.Publish(TeleopTopic, msg);
    }

    void StartDriving()
    {
        var sensor = GetComponentInChildren<RosCameraSensor>();
        var mount = ViewMount != null ? ViewMount : sensor != null ? sensor.transform : null;
        if (mount == null)
        {
            Debug.LogWarning("FirstPersonDrive: no RosCameraSensor or ViewMount on this robot.");
            return;
        }
        if (m_View == null)
        {
            var go = new GameObject("FirstPersonView");
            m_View = go.AddComponent<Camera>();
            m_View.nearClipPlane = 0.01f;
            go.AddComponent<AudioListener>();
        }
        m_View.transform.SetParent(mount, false);
        m_View.fieldOfView = sensor != null ? sensor.VerticalFovDegrees : FallbackVerticalFovDegrees;

        m_Suspended.Clear();
        foreach (var cam in Camera.allCameras)
        {
            if (cam == m_View || cam.targetTexture != null)
            {
                continue;  // render-texture cameras (the ROS camera) keep running
            }
            Suspend(cam);
            Suspend(cam.GetComponent<FreeCam>());
        }
        foreach (var listener in FindObjectsOfType<AudioListener>())
        {
            if (listener.gameObject != m_View.gameObject)
            {
                Suspend(listener);
            }
        }
        m_View.gameObject.SetActive(true);

        m_RobotRenderers = GetComponentsInChildren<Renderer>();
        m_ModeBeforeDriving = m_Controller.mode;
        m_Controller.mode = Assisted ? ControlMode.ROS : ControlMode.Keyboard;
        IsDriving = true;
    }

    void StopDriving()
    {
        if (Assisted)
        {
            PublishTeleop(0f, 0f);
        }
        m_Controller.mode = m_ModeBeforeDriving;
        if (m_View != null)
        {
            m_View.gameObject.SetActive(false);
        }
        foreach (var behaviour in m_Suspended)
        {
            if (behaviour != null)
            {
                behaviour.enabled = true;
            }
        }
        m_Suspended.Clear();
        IsDriving = false;
    }

    void Suspend(Behaviour behaviour)
    {
        if (behaviour != null && behaviour.enabled)
        {
            behaviour.enabled = false;
            m_Suspended.Add(behaviour);
        }
    }

    // Hide the robot from the first-person camera only: other cameras render it as usual.
    void OnBeginCameraRendering(ScriptableRenderContext context, Camera cam)
    {
        if (cam == m_View && HideRobot)
        {
            SetRobotHidden(true);
        }
    }

    void OnEndCameraRendering(ScriptableRenderContext context, Camera cam)
    {
        if (cam == m_View && HideRobot)
        {
            SetRobotHidden(false);
        }
    }

    void SetRobotHidden(bool hidden)
    {
        foreach (var r in m_RobotRenderers)
        {
            if (r != null)
            {
                r.forceRenderingOff = hidden;
            }
        }
    }

    void OnGUI()
    {
        if (!ShowHelp)
        {
            return;
        }
        if (m_HelpStyle == null)
        {
            m_HelpStyle = new GUIStyle(GUI.skin.box) { fontSize = 14, alignment = TextAnchor.MiddleLeft };
            m_HelpStyle.normal.textColor = Color.white;
        }
        var text = IsDriving
            ? string.Format("DRIVING ({0})   W/S drive   A/D turn   {1} {2}   {3} exit",
                Assisted ? "assisted: avoids obstacles" : "direct", AssistKey,
                Assisted ? "direct" : "assisted", ToggleKey)
            : string.Format("{0}: drive (first person)", ToggleKey);
        if (IsDriving && Assisted && m_KeysHeld && m_Controller.SecondsSinceRosCommand > 1f)
        {
            text += "\nNo cmd_vel from ROS: is the ROS side running? (" + AssistKey + " drives directly)";
        }
        var size = m_HelpStyle.CalcSize(new GUIContent(text));
        GUI.Box(new Rect(10, Screen.height - size.y - 16, size.x + 12, size.y + 6), text, m_HelpStyle);
    }
}
