using System.Collections.Generic;
using RosSharp.Control;
using UnityEngine;
using UnityEngine.Rendering;

/// <summary>
/// Press ToggleKey (P) to drive the robot from the keyboard, seen through its own camera.
/// Put it on the robot root next to AGVController. While driving, the Game view renders from the
/// RosCameraSensor's pose with the same vertical FOV, the robot's meshes are hidden from that view
/// only (the ROS camera and physics are unaffected), and cmd_vel from ROS navigation is ignored.
/// Keys: W/S or Up/Down drive, A/D or Left/Right turn. The Game view needs keyboard focus.
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

    public bool IsDriving { get; private set; }

    AGVController m_Controller;
    ControlMode m_ModeBeforeDriving;
    Camera m_View;
    Renderer[] m_RobotRenderers = new Renderer[0];
    // Screen cameras, their FreeCam (also reads WASD/arrows) and audio listeners, off while driving.
    readonly List<Behaviour> m_Suspended = new List<Behaviour>();
    GUIStyle m_HelpStyle;

    void Start()
    {
        m_Controller = GetComponentInChildren<AGVController>();
        if (m_Controller == null)
        {
            Debug.LogWarning("FirstPersonDrive: no AGVController on this robot; driving disabled.");
            enabled = false;
        }
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
        m_Controller.mode = ControlMode.Keyboard;
        IsDriving = true;
    }

    void StopDriving()
    {
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
            ? string.Format("DRIVING   W/S drive   A/D turn   {0} exit", ToggleKey)
            : string.Format("{0}: drive (first person)", ToggleKey);
        var size = m_HelpStyle.CalcSize(new GUIContent(text));
        GUI.Box(new Rect(10, Screen.height - size.y - 16, size.x + 12, size.y + 6), text, m_HelpStyle);
    }
}
