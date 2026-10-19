using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace IntegrationTests
{
    // The warehouse traffic prefabs (Resources/WarehouseMovers) and their physics: the robot's
    // lidar must see each one, each must drive its route, and none may pass through another mover
    // or shove the robot. No ROS needed.
    public class WarehouseMoverTests
    {
        const float k_LidarHeight = 0.132f;     // base_scan above the floor (turtlebot3 model.urdf)
        const float k_Contact = 0.03f;          // deepest overlap allowed between two bodies (m)

        static readonly string[] k_Prefabs = { "Worker", "WorkerWithCart", "PalletJack", "Forklift", "AMR" };
        static readonly string[][] k_Pairs = { new[] { "Worker", "Worker" }, new[] { "Forklift", "Worker" },
            new[] { "PalletJack", "AMR" } };

        readonly List<GameObject> m_Created = new List<GameObject>();
        float m_TimeScale;

        [SetUp]
        public void SetUp() => m_TimeScale = Time.timeScale;

        [UnityTearDown]
        public IEnumerator TearDown()
        {
            Time.timeScale = m_TimeScale;
            foreach (var created in m_Created)
            {
                Object.Destroy(created);
            }
            m_Created.Clear();
            // Its NavMesh was built for this test's floor.
            var navigation = Object.FindObjectOfType<WarehouseNavigation>();
            if (navigation != null)
            {
                Object.Destroy(navigation.gameObject);
            }
            yield return null;
        }

        GameObject Spawn(string name, Vector3 position, float yaw = 0f)
        {
            var prefab = Resources.Load<GameObject>("WarehouseMovers/" + name);
            Assert.IsNotNull(prefab, "missing prefab " + name + ": Tools > Warehouse Movers > Rebuild Prefabs");
            var instance = Object.Instantiate(prefab, position, Quaternion.Euler(0f, yaw, 0f));
            m_Created.Add(instance);
            return instance;
        }

        // A 40 x 40 m floor, top at y = 0, to build a NavMesh on.
        void Floor()
        {
            var floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.transform.localScale = new Vector3(40f, 0.2f, 40f);
            floor.transform.position = new Vector3(0f, -0.1f, 5f);
            m_Created.Add(floor);
        }

        // Deepest overlap between any collider of `a` and any of `b` (0: apart or touching).
        static float Penetration(GameObject a, GameObject b)
        {
            float deepest = 0f;
            var others = b.GetComponentsInChildren<Collider>();
            foreach (var mine in a.GetComponentsInChildren<Collider>())
            {
                foreach (var other in others)
                {
                    if (!mine.bounds.Intersects(other.bounds))
                    {
                        continue;
                    }
                    if (Physics.ComputePenetration(mine, mine.transform.position, mine.transform.rotation,
                            other, other.transform.position, other.transform.rotation, out _, out var depth))
                    {
                        deepest = Mathf.Max(deepest, depth);
                    }
                }
            }
            return deepest;
        }

        static bool BoundsMeet(GameObject a, GameObject b)
        {
            var one = Bounds(a);
            var two = Bounds(b);
            return one.Intersects(two);
        }

        static Bounds Bounds(GameObject go)
        {
            var colliders = go.GetComponentsInChildren<Collider>();
            var bounds = colliders[0].bounds;
            foreach (var collider in colliders)
            {
                bounds.Encapsulate(collider.bounds);
            }
            return bounds;
        }

        [UnityTest]
        public IEnumerator LidarPlaneSeesItFromEverySide([ValueSource(nameof(k_Prefabs))] string name)
        {
            var instance = Spawn(name, Vector3.zero);
            var mover = instance.GetComponent<WarehouseMover>();
            mover.enabled = false;
            yield return new WaitForFixedUpdate();
            Physics.SyncTransforms();
            var half = mover.Footprint / 2f;
            foreach (var direction in new[] { Vector3.forward, Vector3.back, Vector3.left, Vector3.right })
            {
                var across = Vector3.Cross(Vector3.up, direction);
                float span = Mathf.Abs(Vector3.Dot(across, new Vector3(half.x, 0f, half.z)));
                int hits = 0;
                for (float offset = -span; offset <= span; offset += 0.05f)
                {
                    var origin = -5f * direction + offset * across + k_LidarHeight * Vector3.up;
                    if (Physics.Raycast(origin, direction, out var hit, 10f) && hit.transform.IsChildOf(instance.transform))
                    {
                        hits++;
                    }
                }
                Assert.Greater(hits, 0, name + " is invisible to the lidar from " + direction);
            }
        }

        [UnityTest]
        public IEnumerator DrivesItsRouteAndComesBack([ValueSource(nameof(k_Prefabs))] string name)
        {
            Floor();
            var instance = Spawn(name, Vector3.zero);
            var mover = instance.GetComponent<WarehouseMover>();
            mover.Behaviour = WarehouseMover.Mode.Route;
            mover.Route = new List<Vector3> { new Vector3(0f, 0f, 6f) };
            mover.AtEnd = WarehouseMover.RouteEnd.PingPong;
            mover.StopsPerMinute = 0f;
            Time.timeScale = 10f;

            float far = 0f, fastest = 0f, wide = 0f, deadline = Time.time + 60f;
            bool returned = false;
            while (Time.time < deadline && !returned)
            {
                yield return new WaitForFixedUpdate();
                var position = instance.transform.position;
                far = Mathf.Max(far, position.z);
                fastest = Mathf.Max(fastest, mover.Velocity.magnitude);
                wide = Mathf.Max(wide, Mathf.Abs(position.x));
                returned = far > 5.6f && position.z < 0.4f && mover.CurrentSpeed < 0.05f;
            }
            Assert.Greater(far, 5.6f, name + " did not reach the end of its route");
            Assert.IsTrue(returned, name + " did not come back to its start");
            Assert.Less(wide, 0.3f, name + " strayed from its straight route");
            Assert.LessOrEqual(fastest, mover.Speed + 0.05f, name + " exceeded its speed");
            // Back at the start: vehicles reversed and still face the route; people turned round.
            float yaw = Mathf.Abs(Mathf.DeltaAngle(0f, instance.transform.eulerAngles.y));
            Assert.AreEqual(mover.ReverseOnReturn ? 0f : 180f, yaw, 15f, name + " faces the wrong way");
        }

        [UnityTest]
        public IEnumerator MoversPassHeadOnWithoutOverlapping([ValueSource(nameof(k_Pairs))] string[] pair)
        {
            Floor();
            var a = Spawn(pair[0], Vector3.zero);
            var b = Spawn(pair[1], new Vector3(0f, 0f, 12f), 180f);
            foreach (var (go, goal) in new[] { (a, 12f), (b, 0f) })
            {
                var mover = go.GetComponent<WarehouseMover>();
                mover.Behaviour = WarehouseMover.Mode.Route;
                mover.Route = new List<Vector3> { new Vector3(0f, 0f, goal) };
                mover.StopsPerMinute = 0f;
            }
            Time.timeScale = 10f;

            float deepest = 0f, deadline = Time.time + 120f;
            bool passed = false;
            while (Time.time < deadline && !passed)
            {
                yield return new WaitForFixedUpdate();
                if (BoundsMeet(a, b))
                {
                    deepest = Mathf.Max(deepest, Penetration(a, b));
                }
                passed = a.transform.position.z > b.transform.position.z + 1f;
            }
            Assert.Less(deepest, k_Contact, pair[0] + " and " + pair[1] + " went into each other");
            Assert.IsTrue(passed, pair[0] + " and " + pair[1] + " never got past each other");
        }

        // A robot-sized stand-in, as light as the Waffle, at `position`.
        GameObject StandInRobot(Vector3 position)
        {
            var robot = GameObject.CreatePrimitive(PrimitiveType.Cube);
            robot.tag = "robot";
            robot.transform.localScale = new Vector3(0.3f, 0.2f, 0.3f);
            robot.transform.position = position + 0.1f * Vector3.up;
            robot.AddComponent<Rigidbody>().mass = 2.6f;
            m_Created.Add(robot);
            return robot;
        }

        // Smallest gap between the colliders of `a` and `b` (m), 0 when touching.
        static float Clearance(GameObject a, GameObject b)
        {
            float best = float.PositiveInfinity;
            var others = b.GetComponentsInChildren<Collider>();
            foreach (var mine in a.GetComponentsInChildren<Collider>())
            {
                foreach (var other in others)
                {
                    var onOther = other.ClosestPoint(mine.bounds.center);
                    var onMine = mine.ClosestPoint(onOther);
                    best = Mathf.Min(best, Vector3.Distance(onMine, other.ClosestPoint(onMine)));
                }
            }
            return best;
        }

        // Route straight through the robot on open floor: it keeps right and goes round.
        [UnityTest]
        public IEnumerator GetsPastTheRobotWithoutTouchingIt([ValueSource(nameof(k_Prefabs))] string name)
        {
            Floor();
            var robot = StandInRobot(new Vector3(0f, 0f, 6f));
            var instance = Spawn(name, Vector3.zero);
            var mover = instance.GetComponent<WarehouseMover>();
            mover.Behaviour = WarehouseMover.Mode.Route;
            mover.Route = new List<Vector3> { new Vector3(0f, 0f, 12f) };
            mover.AtEnd = WarehouseMover.RouteEnd.Stop;
            mover.StopsPerMinute = 0f;
            Time.timeScale = 10f;

            float closest = float.PositiveInfinity, deadline = Time.time + 60f;
            var start = robot.transform.position;
            while (Time.time < deadline && instance.transform.position.z < 11f)
            {
                yield return new WaitForFixedUpdate();
                closest = Mathf.Min(closest, Clearance(instance, robot));
            }
            Assert.Greater(instance.transform.position.z, 11f, name + " did not get past the robot");
            Assert.Greater(closest, 0.15f, name + " came too close to the robot");
            Assert.Less(Vector3.Distance(start, robot.transform.position), 0.02f, name + " pushed the robot");
        }

        // An aisle too narrow to pass the robot: it stops short, then gives way, never pushing.
        [UnityTest]
        public IEnumerator StopsForTheRobotInANarrowAisle([ValueSource(nameof(k_Prefabs))] string name)
        {
            Floor();
            var prefab = Resources.Load<GameObject>("WarehouseMovers/" + name).GetComponent<WarehouseMover>();
            float half = Mathf.Max(prefab.NavRadius, prefab.Footprint.x / 2f) + 0.15f;
            foreach (var side in new[] { -1f, 1f })
            {
                var wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
                wall.transform.localScale = new Vector3(0.1f, 1f, 30f);
                wall.transform.position = new Vector3(side * (half + 0.05f), 0.5f, 5f);
                m_Created.Add(wall);
            }
            var robot = StandInRobot(new Vector3(0f, 0f, 6f));
            var instance = Spawn(name, Vector3.zero);
            var mover = instance.GetComponent<WarehouseMover>();
            mover.Behaviour = WarehouseMover.Mode.Route;
            mover.Route = new List<Vector3> { new Vector3(0f, 0f, 12f) };
            mover.StopsPerMinute = 0f;
            Time.timeScale = 10f;

            float closest = float.PositiveInfinity, end = Time.time + 30f;
            var start = robot.transform.position;
            while (Time.time < end)
            {
                yield return new WaitForFixedUpdate();
                closest = Mathf.Min(closest, Clearance(instance, robot));
            }
            Assert.Greater(closest, 0.2f, name + " did not stop short of the robot");
            Assert.Less(Vector3.Distance(start, robot.transform.position), 0.02f, name + " pushed the robot");
        }

        // Five kinds roaming the real warehouse for 90 s around the idle robot.
        [UnityTest, Category("Warehouse")]
        public IEnumerator TrafficRoamsTheWarehouseWithoutCollisions()
        {
            yield return SceneManager.LoadSceneAsync("SimpleWarehouseScene");
            yield return null;
            var spawner = Object.FindObjectOfType<MovingObstacleSpawner>();
            Assert.IsNotNull(spawner, "SimpleWarehouseScene has no robot, so no MovingObstacleSpawner");
            var robot = WarehouseNavigation.FindRobotBody();
            yield return new WaitForSeconds(1f);      // let the robot settle on its wheels

            var movers = new List<GameObject>();
            var starts = new List<Vector3>();
            for (int kind = 1; kind < MovingObstacleSpawner.KindCount; kind++)
            {
                var spawned = spawner.SpawnRoaming(kind);
                Assert.IsNotNull(spawned, "no free spot for a roaming " + MovingObstacleSpawner.KindLabel(kind));
                movers.Add(spawned);
                starts.Add(spawned.transform.position);
            }
            var robotStart = robot.position;
            Time.timeScale = 4f;

            float deepest = 0f, end = Time.time + 90f;
            string worst = "";
            int step = 0;
            while (Time.time < end)
            {
                yield return new WaitForFixedUpdate();
                if (++step % 5 != 0)
                {
                    continue;
                }
                for (int i = 0; i < movers.Count; i++)
                {
                    var others = new List<GameObject>(movers.GetRange(i + 1, movers.Count - i - 1)) { robot.gameObject };
                    foreach (var other in others)
                    {
                        if (BoundsMeet(movers[i], other))
                        {
                            float depth = Penetration(movers[i], other);
                            if (depth > deepest)
                            {
                                deepest = depth;
                                worst = movers[i].name + " / " + other.name;
                            }
                        }
                    }
                }
            }
            Time.timeScale = m_TimeScale;
            Assert.Less(deepest, k_Contact, "went into each other: " + worst);
            Assert.Less(Vector3.Distance(robotStart, robot.position), 0.2f, "the robot was pushed");
            for (int i = 0; i < movers.Count; i++)
            {
                var mover = movers[i].GetComponent<WarehouseMover>();
                Assert.Greater(mover.Distance, 15f, movers[i].name + " hardly moved");
                Assert.Greater(Vector3.Distance(starts[i], movers[i].transform.position) +
                    Vector3.Distance(starts[i], mover.Destination), 5f, movers[i].name + " stayed near its start");
            }
            foreach (var mover in movers)
            {
                Object.Destroy(mover);
            }
        }
    }
}
