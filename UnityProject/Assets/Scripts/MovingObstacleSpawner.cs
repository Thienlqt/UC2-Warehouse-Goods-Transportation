using System.Collections.Generic;
using RosSharp.Control;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Play-mode test tool: spawns moving boxes relative to the robot, to test ROS dynamic obstacle
/// avoidance without editing the scene. Added automatically to any scene with an AGVController.
/// Keys (Game view focused): CrossKey (M) a box crossing the robot's path ahead, HeadOnKey (N) a
/// box coming straight at the robot, ClearKey (K) removes them. Each box goes back and forth
/// (MovingObstacle) over the free floor it finds, so it never starts inside a shelf or wall.
/// </summary>
public class MovingObstacleSpawner : MonoBehaviour
{
    public KeyCode CrossKey = KeyCode.M;
    public KeyCode HeadOnKey = KeyCode.N;
    public KeyCode ClearKey = KeyCode.K;
    public bool ShowHelp = true;

    [Header("Box")]
    public Vector3 Size = new Vector3(0.5f, 0.6f, 0.5f);   // must cross the lidar plane
    // A scene object whose mesh the box copies, so the camera detector sees a carton.
    public string LookAlikePrefix = "box001";

    [Header("Crossing (M)")]
    public float CrossAhead = 3f;        // metres ahead of the robot where the box crosses
    public float CrossHalfSpan = 3f;     // metres it travels to either side of the robot's path
    public float CrossSpeed = 0.4f;

    [Header("Head-on (N)")]
    public float HeadOnDistance = 6f;    // start this far ahead (less if a wall is closer)
    public float HeadOnSpeed = 0.3f;
    public float HeadOnStop = 0.6f;      // turn back this far from where the robot was

    const float Margin = 0.05f;          // gap kept to walls and shelves (m)

    readonly List<GameObject> m_Spawned = new List<GameObject>();
    Transform m_Robot;
    Transform m_Heading;
    float m_FloorY;           // floor height under the robot
    MeshFilter m_LookAlike;
    GUIStyle m_Style;
    string m_Message = "";
    float m_MessageUntil;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void AddToScene()
    {
        SceneManager.sceneLoaded -= OnSceneLoaded;
        SceneManager.sceneLoaded += OnSceneLoaded;
        OnSceneLoaded(SceneManager.GetActiveScene(), LoadSceneMode.Single);
    }

    static void OnSceneLoaded(Scene scene, LoadSceneMode mode)
    {
        if (FindObjectOfType<AGVController>() != null && FindObjectOfType<MovingObstacleSpawner>() == null)
        {
            new GameObject("MovingObstacleSpawner").AddComponent<MovingObstacleSpawner>();
        }
    }

    void Start()
    {
        var controller = FindObjectOfType<AGVController>();
        if (controller == null)
        {
            enabled = false;
            return;
        }
        m_Robot = controller.transform;
        // The camera looks along the robot's nose; the controller's own axes may not.
        var camera = m_Robot.GetComponentInChildren<RosCameraSensor>();
        m_Heading = camera != null ? camera.transform : m_Robot;
        m_FloorY = m_Robot.position.y;
        if (LowestHit(m_Robot.position, out var floor))
        {
            m_FloorY = floor.y;
        }
        foreach (var filter in FindObjectsOfType<MeshFilter>())
        {
            if (filter.name.StartsWith(LookAlikePrefix) && filter.sharedMesh != null
                && filter.GetComponent<MeshRenderer>() != null)
            {
                m_LookAlike = filter;
                break;
            }
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(CrossKey))
        {
            SpawnCrossing();
        }
        if (Input.GetKeyDown(HeadOnKey))
        {
            SpawnHeadOn();
        }
        if (Input.GetKeyDown(ClearKey))
        {
            foreach (var box in m_Spawned)
            {
                Destroy(box);
            }
            m_Spawned.Clear();
            Say("Moving boxes removed");
        }
    }

    Vector3 Forward => Vector3.ProjectOnPlane(m_Heading.forward, Vector3.up).normalized;

    void SpawnCrossing()
    {
        var forward = Forward;
        var right = Vector3.Cross(Vector3.up, forward);
        // Try the requested distance first, then nearer and farther ones.
        foreach (var ahead in new[] { CrossAhead, CrossAhead - 0.5f, CrossAhead + 0.5f, CrossAhead - 1f })
        {
            var centre = m_Robot.position + ahead * forward;
            if (!FloorAt(centre, out var floor) || Blocked(floor))
            {
                continue;
            }
            float toLeft = FreeDistance(floor, -right, CrossHalfSpan);
            float toRight = FreeDistance(floor, right, CrossHalfSpan);
            if (toLeft + toRight < 1f)
            {
                continue;
            }
            // Start on the side with more room, so the box enters the path from the open side.
            var start = toLeft >= toRight ? floor - toLeft * right : floor + toRight * right;
            var direction = toLeft >= toRight ? right : -right;
            Spawn(start, direction, toLeft + toRight, CrossSpeed, "crossing");
            return;
        }
        Say("No free floor across the path ahead for a crossing box");
    }

    void SpawnHeadOn()
    {
        var forward = Forward;
        if (!FloorAt(m_Robot.position + 0.6f * forward, out var near))
        {
            Say("No floor ahead of the robot");
            return;
        }
        float free = FreeDistance(near, forward, HeadOnDistance - 0.6f) + 0.6f;
        float travel = free - HeadOnStop;
        if (travel < 1f)
        {
            Say("Not enough free floor ahead for a head-on box");
            return;
        }
        var start = m_Robot.position + free * forward;
        if (!FloorAt(start, out start))
        {
            Say("No floor ahead of the robot");
            return;
        }
        Spawn(start, -forward, travel, HeadOnSpeed, "head-on");
    }

    void Spawn(Vector3 floor, Vector3 direction, float travel, float speed, string kind)
    {
        var box = new GameObject("MovingBox (" + kind + ")");
        box.transform.SetPositionAndRotation(floor, Quaternion.LookRotation(direction, Vector3.up));
        var collider = box.AddComponent<BoxCollider>();
        collider.size = Size;
        collider.center = new Vector3(0f, Size.y / 2f, 0f);
        AddVisual(box.transform);
        box.AddComponent<Rigidbody>().isKinematic = true;
        var mover = box.AddComponent<MovingObstacle>();
        mover.Direction = direction;
        mover.Travel = travel;
        mover.Speed = speed;
        m_Spawned.Add(box);
        Say(string.Format("Spawned a {0} box: {1:F1} m at {2:F1} m/s", kind, travel, speed));
    }

    void AddVisual(Transform box)
    {
        GameObject visual;
        Bounds bounds;
        if (m_LookAlike != null)
        {
            visual = new GameObject("Visual");
            visual.AddComponent<MeshFilter>().sharedMesh = m_LookAlike.sharedMesh;
            visual.AddComponent<MeshRenderer>().sharedMaterials =
                m_LookAlike.GetComponent<MeshRenderer>().sharedMaterials;
            bounds = m_LookAlike.sharedMesh.bounds;
        }
        else
        {
            visual = GameObject.CreatePrimitive(PrimitiveType.Cube);
            Destroy(visual.GetComponent<Collider>());
            bounds = new Bounds(Vector3.zero, Vector3.one);
        }
        visual.transform.SetParent(box, false);
        // Stretch the mesh to the collider, bottom on the floor.
        var scale = new Vector3(Size.x / Mathf.Max(bounds.size.x, 1e-3f),
            Size.y / Mathf.Max(bounds.size.y, 1e-3f), Size.z / Mathf.Max(bounds.size.z, 1e-3f));
        visual.transform.localScale = scale;
        visual.transform.localPosition = new Vector3(-bounds.center.x * scale.x,
            Size.y / 2f - bounds.center.y * scale.y, -bounds.center.z * scale.z);
    }

    bool IsRobotOrBox(Collider collider)
    {
        if (collider.transform.IsChildOf(m_Robot))
        {
            return true;
        }
        foreach (var box in m_Spawned)
        {
            if (box != null && collider.transform.IsChildOf(box.transform))
            {
                return true;
            }
        }
        return false;
    }

    // Floor point below `p`; false over a hole or where something stands on the floor.
    bool FloorAt(Vector3 p, out Vector3 floor)
    {
        bool found = LowestHit(new Vector3(p.x, m_FloorY, p.z), out floor);
        return found && Mathf.Abs(floor.y - m_FloorY) < 0.1f;
    }

    // First surface (not the robot or a spawned box) straight down from 0.5 m above `p`.
    bool LowestHit(Vector3 p, out Vector3 point)
    {
        point = p;
        float best = float.PositiveInfinity;
        foreach (var hit in Physics.RaycastAll(p + 0.5f * Vector3.up, Vector3.down, 1.5f,
                     Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            if (!IsRobotOrBox(hit.collider) && hit.distance < best)
            {
                best = hit.distance;
                point = hit.point;
            }
        }
        return !float.IsInfinity(best);
    }

    Vector3 HalfExtents => new Vector3(Size.x / 2f + Margin, Size.y / 2f - 0.05f, Size.z / 2f + Margin);

    bool Blocked(Vector3 floor)
    {
        var centre = floor + (Size.y / 2f + 0.05f) * Vector3.up;
        foreach (var hit in Physics.OverlapBox(centre, HalfExtents, Quaternion.identity,
                     Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            if (!IsRobotOrBox(hit))
            {
                return true;
            }
        }
        return false;
    }

    // How far the box can slide from `floor` along `direction` (up to `limit`) without touching
    // anything but the robot.
    float FreeDistance(Vector3 floor, Vector3 direction, float limit)
    {
        var centre = floor + (Size.y / 2f + 0.05f) * Vector3.up;
        var rotation = Quaternion.LookRotation(direction, Vector3.up);
        float free = limit;
        foreach (var hit in Physics.BoxCastAll(centre, HalfExtents, direction, rotation, limit,
                     Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            if (!IsRobotOrBox(hit.collider) && hit.distance < free)
            {
                free = hit.distance;
            }
        }
        return Mathf.Max(0f, free - Margin);
    }

    void Say(string message)
    {
        m_Message = message;
        m_MessageUntil = Time.time + 4f;
        Debug.Log("MovingObstacleSpawner: " + message);
    }

    void OnGUI()
    {
        if (!ShowHelp)
        {
            return;
        }
        if (m_Style == null)
        {
            m_Style = new GUIStyle(GUI.skin.box) { fontSize = 14, alignment = TextAnchor.MiddleLeft };
            m_Style.normal.textColor = Color.white;
        }
        m_Spawned.RemoveAll(box => box == null);
        var text = string.Format("{0}: crossing box   {1}: head-on box   {2}: clear ({3} moving)",
            CrossKey, HeadOnKey, ClearKey, m_Spawned.Count);
        if (Time.time < m_MessageUntil)
        {
            text += "\n" + m_Message;
        }
        var size = m_Style.CalcSize(new GUIContent(text));
        GUI.Box(new Rect(Screen.width - size.x - 22, Screen.height - size.y - 16, size.x + 12,
            size.y + 6), text, m_Style);
    }
}
