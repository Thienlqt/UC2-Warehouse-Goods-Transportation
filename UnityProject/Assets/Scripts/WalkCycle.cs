using UnityEngine;

/// <summary>
/// Swings a person's legs (and optionally arms) about their hip and shoulder pivots in step with
/// the WarehouseMover on the same object. The legs carry colliders, so the lidar sees two moving
/// legs as it would on a real person.
/// </summary>
public class WalkCycle : MonoBehaviour
{
    public Transform LeftLeg;
    public Transform RightLeg;
    public Transform LeftArm;       // optional; leave empty when the hands hold a cart or handle
    public Transform RightArm;
    public float StepLength = 0.7f; // metres per step
    public float LegSwing = 25f;    // degrees either side
    public float ArmSwing = 18f;

    WarehouseMover m_Mover;
    Quaternion m_LeftLeg, m_RightLeg, m_LeftArm, m_RightArm;   // rest poses
    float m_Amount;

    void Awake()
    {
        m_Mover = GetComponent<WarehouseMover>();
        m_LeftLeg = Rest(LeftLeg);
        m_RightLeg = Rest(RightLeg);
        m_LeftArm = Rest(LeftArm);
        m_RightArm = Rest(RightArm);
    }

    static Quaternion Rest(Transform limb) => limb != null ? limb.localRotation : Quaternion.identity;

    void Update()
    {
        if (m_Mover == null)
        {
            return;
        }
        // Fade the swing in and out with speed, so a stopping person ends up standing.
        float target = Mathf.Clamp01(m_Mover.CurrentSpeed / 0.3f);
        m_Amount = Mathf.MoveTowards(m_Amount, target, 3f * Time.deltaTime);
        float swing = Mathf.Sin(m_Mover.Distance / StepLength * Mathf.PI) * m_Amount;
        Swing(LeftLeg, m_LeftLeg, swing * LegSwing);
        Swing(RightLeg, m_RightLeg, -swing * LegSwing);
        Swing(LeftArm, m_LeftArm, -swing * ArmSwing);
        Swing(RightArm, m_RightArm, swing * ArmSwing);
    }

    static void Swing(Transform limb, Quaternion rest, float degrees)
    {
        if (limb != null)
        {
            limb.localRotation = rest * Quaternion.Euler(degrees, 0f, 0f);
        }
    }
}
