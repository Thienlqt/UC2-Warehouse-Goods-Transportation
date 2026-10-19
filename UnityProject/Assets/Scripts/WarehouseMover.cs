using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

/// <summary>
/// Drives a warehouse worker or vehicle around the warehouse: a moving obstacle for testing ROS
/// dynamic obstacle avoidance, with the stop-and-go motion real traffic has.
/// - Where: Roam drives to random reachable places across the warehouse, one after another; Route
///   visits the Route points in order. Paths come from a NavMesh (WarehouseNavigation) that keeps
///   NavRadius clear of shelves and walls.
/// - How: it accelerates, brakes to a stop at each destination, waits, and turns towards where it
///   is going before driving on (or drives the way back in reverse, ReverseOnReturn); it can stop
///   at random along the way.
/// - Physics: the body is a real rigidbody (not kinematic) driven by velocity changes, kept upright
///   at its height, so shelves, the robot and other movers block it and it never passes through
///   them. A heavy mover still pushes a light robot it runs into, so it tries not to:
/// - Traffic: NavMeshAgent avoidance steers it round other movers and the robot; it brakes to keep
///   Gap to whatever is ahead, stops pushing on contact, and after Patience seconds blocked it
///   gives way: a roaming mover goes somewhere else, a route mover turns back.
/// The prefabs in Assets/Resources/WarehouseMovers carry one; MovingObstacleSpawner spawns them in
/// Play mode, or place one in a scene.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class WarehouseMover : MonoBehaviour
{
    public enum Mode { Roam, Route }
    public enum RouteEnd { PingPong, Loop, Stop }

    [Tooltip("Roam: drive to random reachable places across the warehouse. Route: visit Route in order.")]
    public Mode Behaviour = Mode.Roam;
    [Tooltip("Route mode: world points visited after the start position; only their horizontal part is used.")]
    public List<Vector3> Route = new List<Vector3>();
    public RouteEnd AtEnd = RouteEnd.PingPong;
    [Tooltip("Roam mode: shortest trip to a new destination (m).")]
    public float MinTrip = 8f;

    [Header("Driving")]
    public float Speed = 1f;                // m/s
    public float Acceleration = 0.6f;       // m/s²; brakes twice as hard
    public float TurnRate = 90f;            // deg/s
    public float PauseSeconds = 1f;         // wait at each destination
    [Tooltip("Route PingPong: drive back in reverse instead of turning round (forklifts, pallet jacks).")]
    public bool ReverseOnReturn;

    [Header("Random stops")]
    public float StopsPerMinute;            // 0: never stop between destinations
    public float StopSeconds = 2f;
    public int Seed;                        // same seed, same stops and destinations

    [Header("Traffic")]
    [Tooltip("Radius of the circle its path keeps clear of shelves and walls (m).")]
    public float NavRadius = 0.4f;
    [Tooltip("NavMeshAgent avoidance priority: lower is more important, others steer round it first.")]
    [Range(0, 99)] public int AvoidancePriority = 50;
    [Tooltip("Brake for the robot ahead as for another mover. Off: it only steers round the robot, " +
             "and may run into it if the robot does not get out of the way.")]
    public bool YieldToRobot = true;
    [Tooltip("Gap it keeps to the robot or another mover ahead (m).")]
    public float Gap = 0.8f;
    [Tooltip("Blocked this long (s), it gives way: goes elsewhere (Roam) or turns back (Route).")]
    public float Patience = 5f;

    [Tooltip("Width (x), height (y) and length (z) around the pivot, which is on the floor.")]
    public Vector3 Footprint = new Vector3(0.5f, 1.8f, 0.5f);

    enum State { Driving, Paused }

    static PhysicMaterial s_Slippery;

    Rigidbody m_Body;
    NavMeshAgent m_Agent;
    bool m_HasNavMesh;
    int m_AgentType;
    readonly List<Vector3> m_Points = new List<Vector3>();
    int m_Target;
    int m_Step = 1;             // +1 forwards through the route, -1 on the way back
    bool m_Reversing;
    State m_State;
    bool m_NewLegAfterPause;    // false: a random stop, carry on to the same destination
    float m_WaitUntil;
    bool m_Stopping;
    Vector3 m_Destination;
    float m_BlockedFor;
    float m_GiveWayAfter;
    float m_ContactUntil;
    Collider m_Passing;         // what it is passing, until that is behind it
    int m_Side;                 // passing on the right (+1) or the left (-1)
    Vector3 m_PassForward;      // travel direction when the pass began
    float m_PassUntil;
    System.Random m_Random;

    public Vector3 Velocity { get; private set; }
    public float CurrentSpeed => Velocity.magnitude;
    // Metres driven so far (for walking animations).
    public float Distance { get; private set; }
    public Vector3 Destination => m_Destination;

    void Start()
    {
        m_Body = GetComponent<Rigidbody>();
        m_Body.isKinematic = false;
        m_Body.useGravity = false;
        // Upright at a fixed height, a hair above the floor: the floor carries no load and has no
        // friction to fight, and the body cannot tip over or climb onto anything.
        m_Body.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotationX
            | RigidbodyConstraints.FreezeRotationZ;
        m_Body.interpolation = RigidbodyInterpolation.Interpolate;
        m_Body.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
        // PhysX freezes rotation about the principal axes of inertia. For a lopsided compound (a
        // cart pushed by a worker) those are tilted, so "freeze X and Z" would not keep it upright.
        // A box inertia on the body's own axes makes them the world's, and stops the swinging legs
        // from changing it.
        float m = m_Body.mass, w = Footprint.x, h = Footprint.y, l = Footprint.z;
        m_Body.centerOfMass = new Vector3(0f, Mathf.Min(0.5f, h / 2f), 0f);
        m_Body.inertiaTensor = m / 12f * new Vector3(h * h + l * l, w * w + l * l, w * w + h * h);
        m_Body.inertiaTensorRotation = Quaternion.identity;
        transform.SetPositionAndRotation(transform.position + 0.01f * Vector3.up,
            Quaternion.Euler(0f, transform.eulerAngles.y, 0f));
        foreach (var part in GetComponentsInChildren<Collider>())
        {
            if (part.sharedMaterial == null)
            {
                part.sharedMaterial = Slippery;
            }
        }

        m_Random = new System.Random(Seed);
        m_GiveWayAfter = Patience;
        var start = transform.position;
        m_Points.Add(start);
        foreach (var point in Route)
        {
            m_Points.Add(new Vector3(point.x, start.y, point.z));
        }
        if ((Behaviour == Mode.Route && m_Points.Count < 2) || Speed <= 0f || Acceleration <= 0f)
        {
            Debug.LogWarning("WarehouseMover: needs a positive Speed and Acceleration, and Route points in Route mode.", this);
            enabled = false;
            return;
        }
        m_HasNavMesh = WarehouseNavigation.TryAgentType(NavRadius, start, out m_AgentType);
        if (m_HasNavMesh)
        {
            CreateAgent();
        }
        else if (Behaviour == Mode.Roam)
        {
            Debug.LogWarning("WarehouseMover: no NavMesh to roam on.", this);
            enabled = false;
            return;
        }
        m_Destination = start;
        m_Target = 0;
        Pause((float)m_Random.NextDouble() * 0.5f, true);
    }

    static PhysicMaterial Slippery
    {
        get
        {
            if (s_Slippery == null)
            {
                s_Slippery = new PhysicMaterial("Slippery")
                {
                    dynamicFriction = 0f, staticFriction = 0f, bounciness = 0f,
                    frictionCombine = PhysicMaterialCombine.Minimum, bounceCombine = PhysicMaterialCombine.Minimum,
                };
            }
            return s_Slippery;
        }
    }

    // The agent plans and avoids; it never moves anything itself (updatePosition off). It lives on
    // a child, created inactive, so it is configured before it looks for its NavMesh.
    void CreateAgent()
    {
        var holder = new GameObject("NavAgent");
        holder.SetActive(false);
        holder.transform.SetParent(transform, false);
        m_Agent = holder.AddComponent<NavMeshAgent>();
        m_Agent.agentTypeID = m_AgentType;
        m_Agent.radius = NavRadius;
        m_Agent.height = Mathf.Min(Footprint.y, 2.4f);
        m_Agent.speed = Speed;
        m_Agent.acceleration = 4f * Acceleration;
        m_Agent.angularSpeed = TurnRate;
        m_Agent.stoppingDistance = 0.25f;
        m_Agent.autoBraking = true;
        m_Agent.updatePosition = false;
        m_Agent.updateRotation = false;
        m_Agent.obstacleAvoidanceType = ObstacleAvoidanceType.HighQualityObstacleAvoidance;
        m_Agent.avoidancePriority = AvoidancePriority;
        holder.SetActive(true);
        if (!m_Agent.isOnNavMesh && WarehouseNavigation.OnFloor(m_AgentType, transform.position, out var point))
        {
            m_Agent.Warp(point);
        }
        if (!m_Agent.isOnNavMesh)
        {
            Debug.LogWarning("WarehouseMover: starts off the NavMesh; drives straight to its points.", this);
            Destroy(holder);
            m_Agent = null;
        }
    }

    bool HasAgent => m_Agent != null && m_Agent.isOnNavMesh;

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        var position = m_Body.position;
        if (HasAgent)
        {
            m_Agent.nextPosition = position;
        }
        var wanted = Vector3.zero;
        if (m_State == State.Paused)
        {
            if (Time.time >= m_WaitUntil && (!m_NewLegAfterPause || NextLeg()))
            {
                m_State = State.Driving;
            }
        }
        else if (Arrived(position))
        {
            Arrive();
        }
        else
        {
            wanted = DesiredVelocity(position);
        }
        Drive(wanted, dt);
    }

    bool Arrived(Vector3 position)
    {
        if (HasAgent)
        {
            return !m_Agent.pathPending && m_Agent.remainingDistance <= m_Agent.stoppingDistance + 0.05f;
        }
        return Flat(m_Destination - position).magnitude < 0.1f;
    }

    Vector3 DesiredVelocity(Vector3 position)
    {
        if (HasAgent)
        {
            return Flat(m_Agent.desiredVelocity);
        }
        var toGo = Flat(m_Destination - position);
        float speed = Mathf.Min(Speed, Mathf.Sqrt(2f * Acceleration * toGo.magnitude));
        return speed * toGo.normalized;
    }

    void Drive(Vector3 wanted, float dt)
    {
        var forward = Flat(transform.forward).normalized;
        var travel = m_Reversing ? -forward : forward;
        float gap = ClearanceAhead(travel, out var ahead);
        if (wanted.sqrMagnitude > 1e-4f)
        {
            wanted = PassOnTheSide(wanted, travel, gap, ahead);
        }
        float yawRate = 0f;
        float speed = 0f;
        if (wanted.sqrMagnitude > 1e-4f)
        {
            // Turn towards where it wants to go; drive only while roughly facing it (no crabbing).
            var heading = m_Reversing ? -wanted : wanted;
            float error = Vector3.SignedAngle(forward, heading, Vector3.up);
            // Never swing the body into something alongside: carry straight on until it is past.
            int beside = SideOfNeighbour();
            if (beside != 0 && Mathf.Sign(error) == beside * (m_Reversing ? -1f : 1f))
            {
                error = 0f;
            }
            yawRate = Mathf.Clamp(3f * error, -TurnRate, TurnRate);
            float align = Mathf.Clamp01((Mathf.Cos(error * Mathf.Deg2Rad) - 0.5f) / 0.5f);
            speed = Mathf.Min(wanted.magnitude, Speed) * align;
        }

        if (m_State == State.Driving && !m_Stopping && StopsPerMinute > 0f
            && m_Random.NextDouble() < StopsPerMinute / 60f * dt)
        {
            m_Stopping = true;
        }
        if (m_Stopping)
        {
            speed = 0f;
        }

        // Keep Gap to the robot or a mover ahead; stop pushing for a moment after contact.
        float wantedSpeed = speed;
        speed = Mathf.Min(speed, Mathf.Sqrt(2f * Acceleration * Mathf.Max(0f, gap - Gap)));
        if (Time.time < m_ContactUntil)
        {
            speed = 0f;
        }

        var current = Flat(m_Body.velocity);
        var change = speed * travel - current;
        float limit = (speed < current.magnitude ? 2f : 1f) * Acceleration * dt;
        m_Body.AddForce(Vector3.ClampMagnitude(change, limit), ForceMode.VelocityChange);
        m_Body.angularVelocity = new Vector3(0f, yawRate * Mathf.Deg2Rad, 0f);

        Velocity = Flat(m_Body.velocity);
        Distance += Velocity.magnitude * dt;
        if (HasAgent)
        {
            m_Agent.velocity = Velocity;    // its avoidance plans with what the body really does
        }

        if (m_Stopping && Velocity.magnitude < 0.02f)
        {
            m_Stopping = false;
            Pause(StopSeconds, false);
        }
        // Held up by traffic, or pushing against something without moving: give way in the end.
        bool blocked = m_State == State.Driving && wantedSpeed > 0.1f && Velocity.magnitude < 0.05f;
        m_BlockedFor = blocked ? m_BlockedFor + dt : 0f;
        if (m_BlockedFor > m_GiveWayAfter)
        {
            GiveWay();
        }
    }

    // Start moving aside this far beyond Gap (m); a long body starts earlier, so it turns less.
    float PassRange => 3f + Footprint.z / 2f;

    // Something coming the other way (or standing) in the lane ahead: keep right, as warehouse
    // traffic does, so two movers meeting head-on both pass on their right; left when the NavMesh
    // has no room on the right. It aims beside the other one with PassClearance between them.
    // Following something going the same way, it just keeps the gap.
    // (NavMeshAgent avoidance only steers round agents of the same size, so this covers the rest.)
    // Gap aimed for alongside (m): more for a long body, whose corners swing out as it turns.
    float PassClearance => 0.35f + 0.1f * Footprint.z;

    Vector3 PassOnTheSide(Vector3 wanted, Vector3 travel, float gap, Collider ahead)
    {
        if (m_Passing == null && ahead != null && gap <= Gap + PassRange
            && Vector3.Dot(VelocityOf(ahead), travel) <= 0.3f)
        {
            var right = Vector3.Cross(Vector3.up, travel);
            m_Side = RoomTowards(right) ? 1 : RoomTowards(-right) ? -1 : 0;
            if (m_Side == 0)
            {
                return wanted;      // no room either side: brake and wait (then give way)
            }
            m_Passing = ahead;
            m_PassForward = travel;
            m_PassUntil = Time.time + 15f;
        }
        if (m_Passing == null)
        {
            return wanted;
        }
        // Aim at a point beside it, on the chosen side and level with its far end, in the frame the
        // pass began in, until all of it is behind the footprint.
        var other = BoundsOf(m_Passing);
        float otherHalf = Mathf.Max(other.extents.x, other.extents.z);
        var relative = Flat(other.center - m_Body.position);
        if (Vector3.Dot(relative, m_PassForward) < -(Footprint.z / 2f + otherHalf + 0.2f) || Time.time > m_PassUntil)
        {
            m_Passing = null;
            return wanted;
        }
        var side = m_Side * Vector3.Cross(Vector3.up, m_PassForward);
        float offset = Footprint.x / 2f + otherHalf + PassClearance;
        var aim = relative + offset * side + (otherHalf + Footprint.z / 2f) * m_PassForward;
        if (Vector3.Dot(aim, m_PassForward) < 0.1f)
        {
            aim = m_PassForward;    // level with it: carry straight on
        }
        // Wanted is the velocity to travel with; reversing, it already points backwards.
        return Mathf.Max(wanted.magnitude, 0.3f * Speed) * aim.normalized;
    }

    // All colliders of the robot or mover `part` belongs to.
    static Bounds BoundsOf(Collider part)
    {
        Transform root = part.transform;
        if (WarehouseNavigation.IsRobot(part) && WarehouseNavigation.Robot != null)
        {
            root = WarehouseNavigation.Robot;
        }
        else
        {
            var mover = part.GetComponentInParent<WarehouseMover>();
            if (mover != null)
            {
                root = mover.transform;
            }
        }
        var bounds = part.bounds;
        foreach (var collider in root.GetComponentsInChildren<Collider>())
        {
            bounds.Encapsulate(collider.bounds);
        }
        return bounds;
    }

    const float k_SideMargin = 0.4f;    // how close alongside counts as beside (m)

    // +1 when the robot or another mover is alongside on the right, -1 on the left, 0 neither.
    int SideOfNeighbour()
    {
        var centre = m_Body.position + Footprint.y / 2f * Vector3.up;
        var halfExtents = new Vector3(Footprint.x / 2f + k_SideMargin, Footprint.y / 2f - 0.05f, Footprint.z / 2f);
        var rotation = Quaternion.Euler(0f, transform.eulerAngles.y, 0f);
        foreach (var other in Physics.OverlapBox(centre, halfExtents, rotation, Physics.DefaultRaycastLayers,
                     QueryTriggerInteraction.Ignore))
        {
            if (other.transform.IsChildOf(transform) || !Counts(other))
            {
                continue;
            }
            var local = Quaternion.Inverse(rotation) * (other.bounds.center - centre);
            return local.x >= 0f ? 1 : -1;
        }
        return 0;
    }

    // The robot (when yielding to it) and other movers: what it keeps its distance from.
    bool Counts(Collider other)
    {
        if (WarehouseNavigation.IsRobot(other))
        {
            return YieldToRobot;
        }
        return other.GetComponentInParent<WarehouseMover>() != null || other.GetComponentInParent<MovingObstacle>() != null;
    }

    bool RoomTowards(Vector3 side)
    {
        if (!HasAgent)
        {
            return true;
        }
        var filter = new NavMeshQueryFilter { agentTypeID = m_AgentType, areaMask = NavMesh.AllAreas };
        var from = m_Agent.nextPosition;
        return !NavMesh.Raycast(from, from + 1.2f * side, out _, filter);
    }

    static Vector3 VelocityOf(Collider other)
    {
        var articulation = other.GetComponentInParent<ArticulationBody>();
        if (articulation != null)
        {
            return Flat(articulation.velocity);
        }
        return other.attachedRigidbody != null ? Flat(other.attachedRigidbody.velocity) : Vector3.zero;
    }

    // Free distance from the front (or back, reversing) of the footprint to the robot or another
    // mover in the lane ahead, up to a little more than the braking distance plus PassRange;
    // `closest` is what is there.
    float ClearanceAhead(Vector3 direction, out Collider closest)
    {
        closest = null;
        float halfLength = Footprint.z / 2f;
        float speed = Velocity.magnitude;
        float look = speed * speed / (2f * Acceleration) + Gap + PassRange;
        var centre = m_Body.position + Footprint.y / 2f * Vector3.up;
        var halfExtents = new Vector3(Footprint.x / 2f, Footprint.y / 2f - 0.05f, 0.05f);
        float clear = float.PositiveInfinity;
        foreach (var hit in Physics.BoxCastAll(centre, halfExtents, direction, Quaternion.LookRotation(direction),
                     halfLength + look, Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore))
        {
            if (hit.collider.transform.IsChildOf(transform))
            {
                continue;
            }
            float distance = Mathf.Max(0f, hit.distance - halfLength);
            if (Counts(hit.collider) && distance < clear)
            {
                clear = distance;
                closest = hit.collider;
            }
        }
        return clear;
    }

    void OnCollisionStay(Collision collision)
    {
        bool robot = WarehouseNavigation.IsRobot(collision.collider);
        if (!(robot && YieldToRobot) && collision.collider.GetComponentInParent<WarehouseMover>() == null)
        {
            return;
        }
        // Only something it is driving into (the contact pushes back against its travel): a
        // scrape along its side or a bump from behind must not stop it.
        var travel = Flat(m_Reversing ? -transform.forward : transform.forward).normalized;
        for (int i = 0; i < collision.contactCount; i++)
        {
            if (Vector3.Dot(Flat(collision.GetContact(i).normal).normalized, travel) < -0.5f)
            {
                m_ContactUntil = Time.time + 0.5f;
                return;
            }
        }
    }

    void Arrive()
    {
        m_Stopping = false;
        Pause(PauseSeconds, true);
    }

    void Pause(float seconds, bool newLeg)
    {
        m_State = State.Paused;
        m_NewLegAfterPause = newLeg;
        m_WaitUntil = Time.time + seconds;
    }

    // Picks the next destination; false to try again after a short wait.
    bool NextLeg()
    {
        m_BlockedFor = 0f;
        m_GiveWayAfter = Patience * (1f + (float)m_Random.NextDouble());   // staggered: not both at once
        if (Behaviour == Mode.Roam)
        {
            m_Reversing = false;
            if (!WarehouseNavigation.RandomDestination(m_AgentType, m_Body.position, MinTrip, m_Random, out var point))
            {
                Pause(2f, true);
                return false;
            }
            return Head(point);
        }
        int next = m_Target + m_Step;
        if (next < 0 || next >= m_Points.Count)
        {
            switch (AtEnd)
            {
                case RouteEnd.Stop:
                    enabled = false;
                    return false;
                case RouteEnd.Loop:
                    next = (next + m_Points.Count) % m_Points.Count;
                    break;
                default:
                    m_Step = -m_Step;
                    next = m_Target + m_Step;
                    break;
            }
        }
        m_Target = next;
        m_Reversing = ReverseOnReturn && m_Step < 0;
        return Head(m_Points[m_Target]);
    }

    bool Head(Vector3 point)
    {
        m_Destination = point;
        if (HasAgent && !m_Agent.SetDestination(point))
        {
            Pause(2f, true);
            return false;
        }
        return true;
    }

    void GiveWay()
    {
        m_BlockedFor = 0f;
        if (Behaviour == Mode.Route)
        {
            // Back to the point it came from (NextLeg steps from m_Target), then on along the route.
            m_Step = -m_Step;
        }
        Pause(0.5f, true);
    }

    void OnEnable()
    {
        if (m_Body != null)
        {
            m_Body.drag = 0f;
        }
    }

    void OnDisable()
    {
        // Stopped for good (Route end, or disabled): it stays a solid body that comes to rest.
        Velocity = Vector3.zero;
        if (m_Body != null)
        {
            m_Body.drag = 5f;
            m_Body.angularVelocity = Vector3.zero;
        }
    }

    static Vector3 Flat(Vector3 v) => new Vector3(v.x, 0f, v.z);

    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.yellow;
        if (Behaviour == Mode.Route)
        {
            var points = new List<Vector3> { Application.isPlaying && m_Points.Count > 0 ? m_Points[0] : transform.position };
            foreach (var point in Route)
            {
                points.Add(new Vector3(point.x, points[0].y, point.z));
            }
            for (int i = 1; i < points.Count; i++)
            {
                Gizmos.DrawLine(points[i - 1], points[i]);
                Gizmos.DrawWireSphere(points[i], 0.1f);
            }
            if (AtEnd == RouteEnd.Loop && points.Count > 2)
            {
                Gizmos.DrawLine(points[points.Count - 1], points[0]);
            }
        }
        if (Application.isPlaying && m_Agent != null && m_Agent.hasPath)
        {
            var corners = m_Agent.path.corners;
            for (int i = 1; i < corners.Length; i++)
            {
                Gizmos.DrawLine(corners[i - 1], corners[i]);
            }
        }
        Gizmos.color = Color.cyan;
        Gizmos.matrix = Matrix4x4.TRS(transform.position, Quaternion.Euler(0f, transform.eulerAngles.y, 0f), Vector3.one);
        Gizmos.DrawWireCube(new Vector3(0f, Footprint.y / 2f, 0f), Footprint);
    }
}
