using System;
using System.Globalization;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// A small runtime panel for sending pickup/dropoff/home requests in the ROS map frame.
/// The ROS delivery_controller owns the FIFO queue and advances only on move_base results.
/// </summary>
public class DeliveryRequestPanel : MonoBehaviour
{
    [Serializable]
    class MapPose
    {
        public float x;
        public float y;
        public float yaw;
    }

    [Serializable]
    class DeliveryRequest
    {
        public string id;
        public MapPose pickup;
        public MapPose dropoff;
        public MapPose home;
    }

    const string RequestTopic = "/delivery/request";
    const string CancelTopic = "/delivery/cancel";
    const string StatusTopic = "/delivery/status";
    const string QueueTopic = "/delivery/queue";

    string m_PickupX = "0";
    string m_PickupY = "0";
    string m_PickupYaw = "0";
    string m_DropoffX = "0";
    string m_DropoffY = "0";
    string m_DropoffYaw = "0";
    string m_HomeX = "0";
    string m_HomeY = "0";
    string m_HomeYaw = "0";
    string m_CancelId = "";
    string m_Status = "Enter map coordinates, then queue a delivery.";
    string m_Queue = "";
    Vector2 m_DetailsScroll;
    ROSConnection m_Ros;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    static void InstallForWarehouse()
    {
        SceneManager.sceneLoaded -= OnSceneLoaded;
        SceneManager.sceneLoaded += OnSceneLoaded;
    }

    static void OnSceneLoaded(Scene scene, LoadSceneMode mode)
    {
        if (scene.name == "SimpleWarehouseScene" && FindObjectOfType<DeliveryRequestPanel>() == null)
        {
            new GameObject("DeliveryRequestPanel").AddComponent<DeliveryRequestPanel>();
        }
    }

    void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<StringMsg>(RequestTopic);
        m_Ros.RegisterPublisher<StringMsg>(CancelTopic);
        m_Ros.Subscribe<StringMsg>(StatusTopic, message => m_Status = message.data);
        m_Ros.Subscribe<StringMsg>(QueueTopic, message => m_Queue = message.data);
    }

    static bool TryPose(string x, string y, string yaw, out MapPose pose)
    {
        pose = null;
        var style = NumberStyles.Float;
        var culture = CultureInfo.InvariantCulture;
        if (!float.TryParse(x, style, culture, out var mapX) ||
            !float.TryParse(y, style, culture, out var mapY) ||
            !float.TryParse(yaw, style, culture, out var mapYaw) ||
            float.IsNaN(mapX) || float.IsInfinity(mapX) ||
            float.IsNaN(mapY) || float.IsInfinity(mapY) ||
            float.IsNaN(mapYaw) || float.IsInfinity(mapYaw))
        {
            return false;
        }
        pose = new MapPose { x = mapX, y = mapY, yaw = mapYaw };
        return true;
    }

    /// <summary>Queue one delivery using ROS map coordinates and yaw in radians.</summary>
    public string EnqueueDelivery(float pickupX, float pickupY, float pickupYaw,
                                  float dropoffX, float dropoffY, float dropoffYaw,
                                  float homeX, float homeY, float homeYaw)
    {
        var id = "delivery-" + Guid.NewGuid().ToString("N").Substring(0, 8);
        var request = new DeliveryRequest
        {
            id = id,
            pickup = new MapPose { x = pickupX, y = pickupY, yaw = pickupYaw },
            dropoff = new MapPose { x = dropoffX, y = dropoffY, yaw = dropoffYaw },
            home = new MapPose { x = homeX, y = homeY, yaw = homeYaw }
        };
        m_Ros.Send(RequestTopic, new StringMsg { data = JsonUtility.ToJson(request) });
        m_Status = "Submitted " + id + "; waiting for ROS acknowledgment.";
        return id;
    }

    public void CancelDelivery(string id)
    {
        m_Ros.Send(CancelTopic, new StringMsg { data = id });
    }

    void OnGUI()
    {
        var wrap = new GUIStyle(GUI.skin.label) { wordWrap = true };
        GUILayout.BeginArea(new Rect(Screen.width - 330, 12, 318, 430),
                            "Delivery requests (map frame)", GUI.skin.window);
        GUILayout.Label("Pickup x / y / yaw (radians)");
        GUILayout.BeginHorizontal();
        m_PickupX = GUILayout.TextField(m_PickupX);
        m_PickupY = GUILayout.TextField(m_PickupY);
        m_PickupYaw = GUILayout.TextField(m_PickupYaw);
        GUILayout.EndHorizontal();
        GUILayout.Label("Dropoff x / y / yaw (radians)");
        GUILayout.BeginHorizontal();
        m_DropoffX = GUILayout.TextField(m_DropoffX);
        m_DropoffY = GUILayout.TextField(m_DropoffY);
        m_DropoffYaw = GUILayout.TextField(m_DropoffYaw);
        GUILayout.EndHorizontal();
        GUILayout.Label("Home x / y / yaw (radians)");
        GUILayout.BeginHorizontal();
        m_HomeX = GUILayout.TextField(m_HomeX);
        m_HomeY = GUILayout.TextField(m_HomeY);
        m_HomeYaw = GUILayout.TextField(m_HomeYaw);
        GUILayout.EndHorizontal();
        if (GUILayout.Button("Queue delivery"))
        {
            if (TryPose(m_PickupX, m_PickupY, m_PickupYaw, out var pickup) &&
                TryPose(m_DropoffX, m_DropoffY, m_DropoffYaw, out var dropoff) &&
                TryPose(m_HomeX, m_HomeY, m_HomeYaw, out var home))
            {
                EnqueueDelivery(pickup.x, pickup.y, pickup.yaw,
                                dropoff.x, dropoff.y, dropoff.yaw,
                                home.x, home.y, home.yaw);
            }
            else
            {
                m_Status = "Use finite numbers with a dot as the decimal separator.";
            }
        }
        GUILayout.BeginHorizontal();
        m_CancelId = GUILayout.TextField(m_CancelId);
        if (GUILayout.Button("Cancel ID", GUILayout.Width(90)) && !string.IsNullOrWhiteSpace(m_CancelId))
        {
            CancelDelivery(m_CancelId.Trim());
        }
        GUILayout.EndHorizontal();
        m_DetailsScroll = GUILayout.BeginScrollView(m_DetailsScroll, GUILayout.Height(130));
        GUILayout.Label("Status: " + m_Status, wrap);
        GUILayout.Label("Queue: " + m_Queue, wrap);
        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }
}
