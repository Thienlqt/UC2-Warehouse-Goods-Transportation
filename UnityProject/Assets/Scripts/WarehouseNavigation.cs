using System.Collections.Generic;
using RosSharp.Control;
using UnityEngine;
using UnityEngine.AI;

/// <summary>
/// Navigation for WarehouseMover in Play mode, created by the first mover that needs it.
/// - Builds a NavMesh of the scene's colliders for each mover size on first use, so a path keeps
///   the mover's NavRadius clear of shelves and walls. The robot and the movers themselves are
///   left out: they move.
/// - Gives the robot a NavMeshObstacle, so movers' NavMeshAgent avoidance steers round it as it
///   does round other movers.
/// - Picks reachable destinations across the warehouse floor.
/// </summary>
public class WarehouseNavigation : MonoBehaviour
{
    const float k_AgentHeight = 2.4f;   // tallest mover (forklift overhead guard); one height for all
    const float k_AgentClimb = 0.05f;   // pallets and cartons on the floor are obstacles, not steps
    const float k_Reach = 60f;          // build at most this far around the robot (m)

    static WarehouseNavigation s_Instance;

    // Radius (cm) -> agent type, or null when its NavMesh could not be built. Agent type ids are
    // arbitrary ints, negative ones included.
    readonly Dictionary<int, int?> m_AgentTypes = new Dictionary<int, int?>();
    readonly List<NavMeshDataInstance> m_Data = new List<NavMeshDataInstance>();
    readonly List<int> m_Settings = new List<int>();
    Bounds m_Bounds;
    float m_FloorY;

    /// <summary>The robot's moving body (its root articulation), or null without a robot.</summary>
    public static Transform Robot => Instance.m_Robot;
    Transform m_Robot;

    /// <summary>Floor height under the robot (or under the first mover without one).</summary>
    public static float FloorY => Instance.m_FloorY;

    static WarehouseNavigation Instance
    {
        get
        {
            if (s_Instance == null)
            {
                s_Instance = new GameObject("WarehouseNavigation").AddComponent<WarehouseNavigation>();
            }
            return s_Instance;
        }
    }

    /// <summary>The transform that moves with the robot: the AGVController sits on a root that
    /// stays put while the root articulation (base_footprint) drives.</summary>
    public static Transform FindRobotBody()
    {
        var controller = FindObjectOfType<AGVController>();
        if (controller == null)
        {
            return null;
        }
        foreach (var body in controller.GetComponentsInChildren<ArticulationBody>())
        {
            if (body.isRoot)
            {
                return body.transform;
            }
        }
        return controller.transform;
    }

    /// <summary>True for the robot's colliders (and anything tagged "robot").</summary>
    public static bool IsRobot(Component part)
    {
        if (part == null)
        {
            return false;
        }
        var robot = Instance.m_Robot;
        if (robot != null && part.transform.IsChildOf(robot))
        {
            return true;
        }
        for (var t = part.transform; t != null; t = t.parent)
        {
            if (t.CompareTag("robot"))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>True for colliders of things that move: the robot and the movers.</summary>
    public static bool IsMoving(Component part)
    {
        return part != null && (IsRobot(part) || part.GetComponentInParent<WarehouseMover>() != null
            || part.GetComponentInParent<MovingObstacle>() != null || part.GetComponentInParent<ArticulationBody>() != null);
    }

    /// <summary>The NavMesh agent type for movers of this radius, building its NavMesh first if
    /// needed; false when there is no floor to build on. `near` locates the floor when the scene
    /// has no robot.</summary>
    public static bool TryAgentType(float radius, Vector3 near, out int agentType)
    {
        var id = Instance.GetAgentType(radius, near);
        agentType = id ?? 0;
        return id.HasValue;
    }

    /// <summary>A random point on the floor reachable from `from`, at least `minDistance` away.</summary>
    public static bool RandomDestination(int agentType, Vector3 from, float minDistance, System.Random random,
        out Vector3 destination)
    {
        destination = from;
        var self = Instance;
        var filter = new NavMeshQueryFilter { agentTypeID = agentType, areaMask = NavMesh.AllAreas };
        if (!NavMesh.SamplePosition(from, out var start, 1f, filter))
        {
            return false;
        }
        var path = new NavMeshPath();
        var b = self.m_Bounds;
        for (int i = 0; i < 40; i++)
        {
            var p = new Vector3(Mathf.Lerp(b.min.x, b.max.x, (float)random.NextDouble()), self.m_FloorY,
                Mathf.Lerp(b.min.z, b.max.z, (float)random.NextDouble()));
            if (!OnFloor(agentType, p, out var point) || Flat(point - from).magnitude < minDistance)
            {
                continue;
            }
            if (NavMesh.CalculatePath(start.position, point, filter, path) && path.status == NavMeshPathStatus.PathComplete)
            {
                destination = point;
                return true;
            }
        }
        return false;
    }

    /// <summary>The nearest floor NavMesh point within 0.5 m of `p` (not the top of a shelf).</summary>
    public static bool OnFloor(int agentType, Vector3 p, out Vector3 point)
    {
        point = p;
        var filter = new NavMeshQueryFilter { agentTypeID = agentType, areaMask = NavMesh.AllAreas };
        if (!NavMesh.SamplePosition(new Vector3(p.x, Instance.m_FloorY, p.z), out var hit, 0.5f, filter)
            || Mathf.Abs(hit.position.y - Instance.m_FloorY) > 0.2f)
        {
            return false;
        }
        point = hit.position;
        return true;
    }

    static Vector3 Flat(Vector3 v) => new Vector3(v.x, 0f, v.z);

    void Awake()
    {
        m_Robot = FindRobotBody();
        if (m_Robot != null && m_Robot.GetComponent<NavMeshObstacle>() == null)
        {
            // Not carving: a moving obstacle the agents steer round. The Waffle is about 0.3 x 0.3 m.
            var obstacle = m_Robot.gameObject.AddComponent<NavMeshObstacle>();
            obstacle.shape = NavMeshObstacleShape.Capsule;
            obstacle.radius = 0.25f;
            obstacle.height = 0.4f;
            obstacle.center = new Vector3(0f, 0.2f, 0f);
            obstacle.carving = false;
        }
    }

    int? GetAgentType(float radius, Vector3 near)
    {
        int key = Mathf.RoundToInt(radius * 100f);
        if (m_AgentTypes.TryGetValue(key, out var id))
        {
            return id;
        }
        if (m_AgentTypes.Count == 0 && !FindFloor(near))
        {
            Debug.LogWarning("WarehouseNavigation: no floor found; movers drive straight to their points.");
            return null;
        }
        var settings = NavMesh.CreateSettings();
        settings.agentRadius = radius;
        settings.agentHeight = k_AgentHeight;
        settings.agentClimb = k_AgentClimb;
        settings.agentSlope = 15f;
        settings.minRegionArea = 1f;
        settings.overrideVoxelSize = true;
        settings.voxelSize = Mathf.Min(radius / 3f, 0.1f);

        var started = Time.realtimeSinceStartup;
        var sources = new List<NavMeshBuildSource>();
        NavMeshBuilder.CollectSources(m_Bounds, ~0, NavMeshCollectGeometry.PhysicsColliders, 0,
            new List<NavMeshBuildMarkup>(), sources);
        sources.RemoveAll(s => s.component == null || IsMoving(s.component)
            || (s.component is Collider c && c.isTrigger));
        var data = NavMeshBuilder.BuildNavMeshData(settings, sources, m_Bounds, Vector3.zero, Quaternion.identity);
        if (data == null)
        {
            NavMesh.RemoveSettings(settings.agentTypeID);
            return m_AgentTypes[key] = null;
        }
        m_Data.Add(NavMesh.AddNavMeshData(data));
        m_Settings.Add(settings.agentTypeID);
        Debug.LogFormat("WarehouseNavigation: built the NavMesh for {0:F2} m movers in {1:F1} s ({2} colliders)",
            radius, Time.realtimeSinceStartup - started, sources.Count);
        return m_AgentTypes[key] = settings.agentTypeID;
    }

    // The floor height and the area to build: every static collider, at most k_Reach from the robot.
    bool FindFloor(Vector3 near)
    {
        Physics.SyncTransforms();      // colliders created this frame (autoSyncTransforms is off)
        var origin = m_Robot != null ? m_Robot.position : near;
        bool found = false;
        float best = float.PositiveInfinity;
        foreach (var hit in Physics.RaycastAll(origin + 0.5f * Vector3.up, Vector3.down, 2f,
                     Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            if (!IsMoving(hit.collider) && hit.distance < best)
            {
                best = hit.distance;
                m_FloorY = hit.point.y;
                found = true;
            }
        }
        if (!found)
        {
            return false;
        }
        var area = new Bounds(new Vector3(origin.x, m_FloorY, origin.z), Vector3.zero);
        bool any = false;
        foreach (var collider in FindObjectsOfType<Collider>())
        {
            if (collider.isTrigger || IsMoving(collider))
            {
                continue;
            }
            area.Encapsulate(collider.bounds);
            any = true;
        }
        if (!any)
        {
            return false;
        }
        var min = Vector3.Max(area.min, origin - k_Reach * Vector3.one);
        var max = Vector3.Min(area.max, origin + k_Reach * Vector3.one);
        min.y = m_FloorY - 0.3f;
        max.y = m_FloorY + k_AgentHeight + 0.6f;
        m_Bounds.SetMinMax(min, max);
        return true;
    }

    void OnDestroy()
    {
        foreach (var data in m_Data)
        {
            NavMesh.RemoveNavMeshData(data);
        }
        foreach (var id in m_Settings)
        {
            NavMesh.RemoveSettings(id);
        }
        if (s_Instance == this)
        {
            s_Instance = null;
        }
    }
}
