using System.Collections.Generic;
using RosSharp.Control;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Play-mode test tool: spawns moving obstacles to test ROS dynamic obstacle avoidance without
/// editing the scene. Added automatically to any scene with an AGVController.
/// Keys (Game view focused): KindKey (B) picks what to spawn (a box, or warehouse traffic: worker,
/// worker with cart, pallet jack, forklift, AMR); RoamKey (T) spawns it somewhere free near the
/// robot to roam the whole warehouse; CrossKey (M) spawns it crossing the robot's path ahead and
/// HeadOnKey (N) coming straight at the robot, both going back and forth; ClearKey (K) removes
/// them. TrafficOnStart roams that many from the start.
/// Everything spawned is a WarehouseMover with a real rigidbody: it never starts inside a shelf,
/// wall, the robot or another mover, and cannot pass through any of them. The warehouse traffic
/// is the prefabs in Resources/WarehouseMovers (built by Tools > Warehouse Movers > Rebuild Prefabs).
/// </summary>
public class MovingObstacleSpawner : MonoBehaviour
{
    public KeyCode KindKey = KeyCode.B;
    public KeyCode RoamKey = KeyCode.T;
    public KeyCode CrossKey = KeyCode.M;
    public KeyCode HeadOnKey = KeyCode.N;
    public KeyCode ClearKey = KeyCode.K;
    public bool ShowHelp = true;
    [Tooltip("Spawned movers brake for the robot (WarehouseMover.YieldToRobot). Off: they only " +
             "steer round it, and push it if it does not get out of the way.")]
    public bool YieldToRobot = true;

    [Header("Box")]
    public Vector3 Size = new Vector3(0.5f, 0.6f, 0.5f);   // must cross the lidar plane
    // A scene object whose mesh the box copies, so the camera detector sees a carton.
    public string LookAlikePrefix = "box001";

    // Label shown on screen and prefab in Resources/WarehouseMovers (none: a box).
    static readonly (string Label, string Prefab)[] k_Kinds =
    {
        ("box", null), ("worker", "Worker"), ("worker with cart", "WorkerWithCart"),
        ("pallet jack", "PalletJack"), ("forklift", "Forklift"), ("AMR", "AMR"),
    };

    [Header("Roaming (T)")]
    [Tooltip("Roaming traffic spawned at the start, kinds in turn (no boxes).")]
    public int TrafficOnStart;
    public float RoamNear = 3f;          // spawn at least this far from the robot (m)
    public float RoamFar = 15f;          // and at most this far
    public float SpawnClearance = 0.5f;  // gap to the robot and other movers at spawn (m)

    [Header("Crossing (M)")]
    public float CrossAhead = 3f;        // metres ahead of the robot where it crosses
    public float CrossHalfSpan = 3f;     // metres it travels to either side of the robot's path
    public float CrossSpeed = 0.4f;      // boxes; warehouse traffic keeps its prefab's speed

    [Header("Head-on (N)")]
    public float HeadOnDistance = 6f;    // start this far ahead (less if a wall is closer)
    public float HeadOnSpeed = 0.3f;     // boxes; warehouse traffic keeps its prefab's speed
    public float HeadOnGap = 0.35f;      // its front turns back this far from where the robot was

    const float Margin = 0.05f;          // gap kept to walls and shelves (m)

    readonly List<GameObject> m_Spawned = new List<GameObject>();
    int m_Kind;
    Transform m_Robot;        // the robot's moving body
    Transform m_Heading;
    float m_FloorY;           // floor height under the robot
    MeshFilter m_LookAlike;
    GUIStyle m_Style;
    string m_Message = "";
    float m_MessageUntil;

    public static int KindCount => k_Kinds.Length;
    public static string KindLabel(int kind) => k_Kinds[kind].Label;
    public IReadOnlyList<GameObject> Spawned => m_Spawned;

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
        m_Robot = WarehouseNavigation.FindRobotBody();
        if (m_Robot == null)
        {
            enabled = false;
            return;
        }
        // The camera looks along the robot's nose; the body's own axes may not.
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
        for (int i = 0; i < TrafficOnStart; i++)
        {
            SpawnRoaming(1 + i % (k_Kinds.Length - 1));
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KindKey))
        {
            m_Kind = (m_Kind + 1) % k_Kinds.Length;
            Say("Next spawn: " + k_Kinds[m_Kind].Label);
        }
        if (Input.GetKeyDown(RoamKey))
        {
            SpawnRoaming(m_Kind);
        }
        if (Input.GetKeyDown(CrossKey))
        {
            SpawnCrossing(m_Kind);
        }
        if (Input.GetKeyDown(HeadOnKey))
        {
            SpawnHeadOn(m_Kind);
        }
        if (Input.GetKeyDown(ClearKey))
        {
            foreach (var spawned in m_Spawned)
            {
                Destroy(spawned);
            }
            m_Spawned.Clear();
            Say("Moving obstacles removed");
        }
    }

    Vector3 Forward => Vector3.ProjectOnPlane(m_Heading.forward, Vector3.up).normalized;

    // The kind's mover settings: its prefab's, or a box's. Null when the prefab is missing.
    WarehouseMover Template(int kind)
    {
        var prefabName = k_Kinds[kind].Prefab;
        if (prefabName == null)
        {
            return null;
        }
        var prefab = Resources.Load<GameObject>("WarehouseMovers/" + prefabName);
        var mover = prefab != null ? prefab.GetComponent<WarehouseMover>() : null;
        if (mover == null)
        {
            Say("Missing prefab Resources/WarehouseMovers/" + prefabName + ": run Tools > Warehouse Movers > Rebuild Prefabs");
        }
        return mover;
    }

    bool Available(int kind) => k_Kinds[kind].Prefab == null || Template(kind) != null;

    // Half width, height and half length; a mover that turns on the spot needs its whole turning
    // circle clear.
    Vector3 Extents(int kind, bool turns)
    {
        var template = Template(kind);
        var size = template != null ? template.Footprint : Size;
        if (turns)
        {
            float diameter = new Vector2(size.x, size.z).magnitude;
            size = new Vector3(diameter, size.y, diameter);
        }
        return size / 2f;
    }

    /// <summary>Spawns `kind` somewhere free RoamNear-RoamFar from the robot, roaming the
    /// warehouse. Null when there is no free spot.</summary>
    public GameObject SpawnRoaming(int kind)
    {
        if (!Available(kind))
        {
            return null;
        }
        Physics.SyncTransforms();
        var template = Template(kind);
        float radius = template != null ? template.NavRadius : NavRadiusForBox;
        bool navigable = WarehouseNavigation.TryAgentType(radius, m_Robot.position, out var agentType);
        var extents = Extents(kind, true);
        for (int i = 0; i < 80; i++)
        {
            float angle = Random.Range(0f, 2f * Mathf.PI);
            var p = m_Robot.position + Random.Range(RoamNear, RoamFar) * new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle));
            if (navigable && !WarehouseNavigation.OnFloor(agentType, p, out p))
            {
                continue;
            }
            var facing = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f) * Vector3.forward;
            if (!FloorAt(p, out var floor) || Blocked(floor, facing, extents))
            {
                continue;
            }
            var mover = Spawn(kind, floor, facing, "roaming");
            mover.Behaviour = WarehouseMover.Mode.Roam;
            mover.Seed = Random.Range(0, int.MaxValue);
            Say(string.Format("Spawned a roaming {0} {1:F1} m away", k_Kinds[kind].Label,
                Vector3.Distance(floor, m_Robot.position)));
            return mover.gameObject;
        }
        Say("No free floor near the robot for a roaming " + k_Kinds[kind].Label);
        return null;
    }

    public GameObject SpawnCrossing(int kind)
    {
        if (!Available(kind))
        {
            return null;
        }
        Physics.SyncTransforms();
        var forward = Forward;
        var right = Vector3.Cross(Vector3.up, forward);
        var template = Template(kind);
        var extents = Extents(kind, template != null && !template.ReverseOnReturn);
        // Try the requested distance first, then nearer and farther ones.
        foreach (var ahead in new[] { CrossAhead, CrossAhead - 0.5f, CrossAhead + 0.5f, CrossAhead - 1f })
        {
            var centre = m_Robot.position + ahead * forward;
            if (!FloorAt(centre, out var floor) || Blocked(floor, right, extents))
            {
                continue;
            }
            float toLeft = FreeDistance(floor, -right, CrossHalfSpan, extents);
            float toRight = FreeDistance(floor, right, CrossHalfSpan, extents);
            if (toLeft + toRight < 1f)
            {
                continue;
            }
            // Start on the side with more room, so it enters the path from the open side.
            var start = toLeft >= toRight ? floor - toLeft * right : floor + toRight * right;
            var direction = toLeft >= toRight ? right : -right;
            if (Blocked(start, direction, extents))
            {
                continue;
            }
            return SpawnShuttle(kind, start, direction, toLeft + toRight, CrossSpeed, "crossing");
        }
        Say("No free floor across the path ahead for a crossing " + k_Kinds[kind].Label);
        return null;
    }

    public GameObject SpawnHeadOn(int kind)
    {
        if (!Available(kind))
        {
            return null;
        }
        Physics.SyncTransforms();
        var forward = Forward;
        var template = Template(kind);
        var extents = Extents(kind, template != null && !template.ReverseOnReturn);
        if (!FloorAt(m_Robot.position + 0.6f * forward, out var near))
        {
            Say("No floor ahead of the robot");
            return null;
        }
        float free = FreeDistance(near, forward, HeadOnDistance - 0.6f, extents) + 0.6f;
        float travel = free - HeadOnGap - extents.z;
        var start = m_Robot.position + free * forward;
        if (travel < 1f || !FloorAt(start, out start) || Blocked(start, -forward, extents))
        {
            Say("Not enough free floor ahead for a head-on " + k_Kinds[kind].Label);
            return null;
        }
        return SpawnShuttle(kind, start, -forward, travel, HeadOnSpeed, "head-on");
    }

    GameObject SpawnShuttle(int kind, Vector3 start, Vector3 direction, float travel, float boxSpeed, string how)
    {
        var mover = Spawn(kind, start, direction, how);
        mover.Behaviour = WarehouseMover.Mode.Route;
        mover.Route = new List<Vector3> { start + travel * direction };
        mover.AtEnd = WarehouseMover.RouteEnd.PingPong;
        if (k_Kinds[kind].Prefab == null)
        {
            mover.Speed = boxSpeed;
        }
        Say(string.Format("Spawned a {0} {1}: {2:F1} m at {3:F1} m/s", how, k_Kinds[kind].Label, travel, mover.Speed));
        return mover.gameObject;
    }

    const float NavRadiusForBox = 0.4f;

    WarehouseMover Spawn(int kind, Vector3 floor, Vector3 direction, string how)
    {
        var rotation = Quaternion.LookRotation(direction, Vector3.up);
        WarehouseMover mover;
        var template = Template(kind);
        if (template != null)
        {
            var traffic = Instantiate(template.gameObject, floor, rotation);
            traffic.name = template.name + " (" + how + ")";
            mover = traffic.GetComponent<WarehouseMover>();
        }
        else
        {
            var box = new GameObject("MovingBox (" + how + ")");
            box.transform.SetPositionAndRotation(floor, rotation);
            var collider = box.AddComponent<BoxCollider>();
            collider.size = Size;
            collider.center = new Vector3(0f, Size.y / 2f, 0f);
            AddVisual(box.transform);
            box.AddComponent<Rigidbody>().mass = 20f;
            mover = box.AddComponent<WarehouseMover>();
            mover.Footprint = Size;
            mover.NavRadius = NavRadiusForBox;
            mover.Speed = 0.6f;
            mover.Acceleration = 0.6f;
            mover.ReverseOnReturn = true;    // a box slides back rather than turning round
        }
        mover.YieldToRobot = YieldToRobot;
        m_Spawned.Add(mover.gameObject);
        return mover;
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

    // Floor point below `p`; false over a hole or where something stands on the floor.
    bool FloorAt(Vector3 p, out Vector3 floor)
    {
        bool found = LowestHit(new Vector3(p.x, m_FloorY, p.z), out floor);
        return found && Mathf.Abs(floor.y - m_FloorY) < 0.1f;
    }

    // First surface (not the robot or a mover) straight down from 0.5 m above `p`.
    bool LowestHit(Vector3 p, out Vector3 point)
    {
        point = p;
        float best = float.PositiveInfinity;
        foreach (var hit in Physics.RaycastAll(p + 0.5f * Vector3.up, Vector3.down, 1.5f,
                     Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            if (!WarehouseNavigation.IsMoving(hit.collider) && hit.distance < best)
            {
                best = hit.distance;
                point = hit.point;
            }
        }
        return !float.IsInfinity(best);
    }

    // A box of `extents` plus `margin`, its bottom 0.05 m above the floor, facing `direction`.
    static Vector3 Grow(Vector3 extents, float margin) =>
        new Vector3(extents.x + margin, extents.y - 0.05f, extents.z + margin);

    // Something in the way at `floor`: shelves and walls within Margin, the robot and other
    // movers within SpawnClearance.
    bool Blocked(Vector3 floor, Vector3 direction, Vector3 extents)
    {
        var centre = floor + (extents.y + 0.05f) * Vector3.up;
        var rotation = Quaternion.LookRotation(direction, Vector3.up);
        foreach (var hit in Physics.OverlapBox(centre, Grow(extents, Margin), rotation,
                     Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            if (!WarehouseNavigation.IsMoving(hit))
            {
                return true;
            }
        }
        foreach (var hit in Physics.OverlapBox(centre, Grow(extents, SpawnClearance), rotation,
                     Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            if (WarehouseNavigation.IsMoving(hit))
            {
                return true;
            }
        }
        return false;
    }

    // How far a box of `extents` can slide from `floor` along `direction` (up to `limit`) without
    // touching anything but the robot (its route may cross the robot's path, not other movers).
    float FreeDistance(Vector3 floor, Vector3 direction, float limit, Vector3 extents)
    {
        var centre = floor + (extents.y + 0.05f) * Vector3.up;
        var rotation = Quaternion.LookRotation(direction, Vector3.up);
        float free = limit;
        foreach (var hit in Physics.BoxCastAll(centre, Grow(extents, Margin), direction, rotation, limit,
                     Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            if (!WarehouseNavigation.IsRobot(hit.collider) && hit.distance < free)
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
        m_Spawned.RemoveAll(spawned => spawned == null);
        var text = string.Format("{0}: {1}   {2}: roam   {3}: crossing   {4}: head-on   {5}: clear ({6} moving)",
            KindKey, k_Kinds[m_Kind].Label, RoamKey, CrossKey, HeadOnKey, ClearKey, m_Spawned.Count);
        if (Time.time < m_MessageUntil)
        {
            text += "\n" + m_Message;
        }
        var size = m_Style.CalcSize(new GUIContent(text));
        GUI.Box(new Rect(Screen.width - size.x - 22, Screen.height - size.y - 16, size.x + 12,
            size.y + 6), text, m_Style);
    }
}
