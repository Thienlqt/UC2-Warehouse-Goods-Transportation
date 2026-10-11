using UnityEngine;

/// <summary>
/// Moves this object back and forth along a straight line at constant speed: a moving obstacle
/// for testing ROS dynamic obstacle avoidance. The robot's lidar and camera see its collider and
/// renderers like any other object.
/// Place it in a scene by hand (it travels Travel metres along Direction from where it starts),
/// or spawn one at runtime with MovingObstacleSpawner. The body is kinematic, so it is not pushed
/// by the robot; it pushes the robot if the robot fails to get out of its way.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class MovingObstacle : MonoBehaviour
{
    public Vector3 Direction = Vector3.right;   // world frame; only its horizontal part is used
    public float Travel = 4f;                   // metres before turning back
    public float Speed = 0.4f;                  // m/s
    public float PauseSeconds = 1f;             // wait at each end before turning back
    public bool Loop = true;                    // false: stop for good at the far end

    Rigidbody m_Body;
    Vector3 m_Start;
    Vector3 m_Unit;
    float m_Position;       // metres from the start along m_Unit
    float m_Sign = 1f;
    float m_PausedUntil;

    public Vector3 Velocity { get; private set; }

    void Start()
    {
        m_Body = GetComponent<Rigidbody>();
        m_Body.isKinematic = true;
        m_Body.interpolation = RigidbodyInterpolation.Interpolate;
        m_Start = transform.position;
        m_Unit = Vector3.ProjectOnPlane(Direction, Vector3.up).normalized;
        if (m_Unit == Vector3.zero || Travel <= 0f)
        {
            Debug.LogWarning("MovingObstacle: needs a horizontal Direction and a positive Travel.", this);
            enabled = false;
        }
    }

    void FixedUpdate()
    {
        Velocity = Vector3.zero;
        if (Time.time < m_PausedUntil)
        {
            return;
        }
        m_Position += m_Sign * Speed * Time.fixedDeltaTime;
        if (m_Position >= Travel || m_Position <= 0f)
        {
            m_Position = Mathf.Clamp(m_Position, 0f, Travel);
            if (m_Position >= Travel && !Loop)
            {
                enabled = false;
            }
            m_Sign = -m_Sign;
            m_PausedUntil = Time.time + PauseSeconds;
        }
        else
        {
            Velocity = m_Sign * Speed * m_Unit;
        }
        m_Body.MovePosition(m_Start + m_Position * m_Unit);
    }

    void OnDrawGizmosSelected()
    {
        var start = Application.isPlaying ? m_Start : transform.position;
        var unit = Vector3.ProjectOnPlane(Direction, Vector3.up).normalized;
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(start, start + Travel * unit);
    }
}
