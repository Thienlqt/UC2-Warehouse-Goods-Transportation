using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Builds the warehouse traffic prefabs (Assets/Resources/WarehouseMovers) out of Unity primitives:
/// a walking worker, a worker pushing a platform cart, a pallet jack with its operator, a loaded
/// forklift and an AMR. Each has a kinematic Rigidbody and a WarehouseMover; MovingObstacleSpawner
/// spawns them in Play mode. Sizes are real-world metres, +z is forward and the pivot is on the
/// floor at the centre of the footprint. Every model has solid colliders across the lidar plane
/// (0.13 m above the floor); small trim (shoes, wheels, lights) has none. The colliders use the
/// frictionless Assets/Materials/NoFriction material, and the Rigidbody is dynamic with a real-world
/// mass, so movers push each other and the robot like the real thing (see WarehouseMover).
/// Runs once when the prefabs are missing; Tools > Warehouse Movers > Rebuild Prefabs rebuilds them
/// (materials and prefabs keep their GUIDs).
/// </summary>
public static class WarehouseMoverBuilder
{
    const string PrefabFolder = "Assets/Resources/WarehouseMovers";
    const string MaterialFolder = "Assets/Materials/WarehouseMovers";
    const string FrictionlessPath = "Assets/Materials/NoFriction.physicMaterial";
    static readonly string[] Names = { "Worker", "WorkerWithCart", "PalletJack", "Forklift", "AMR" };

    enum Shape { Box, Cylinder, Capsule, Sphere }

    static readonly Dictionary<string, Material> s_Materials = new Dictionary<string, Material>();

    [InitializeOnLoadMethod]
    static void BuildIfMissing()
    {
        if (Application.isBatchMode || !Missing())
        {
            return;
        }
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            Debug.Log("WarehouseMoverBuilder: prefabs missing; building them when Play mode ends");
            EditorApplication.playModeStateChanged += AfterPlay;
            return;
        }
        EditorApplication.delayCall += () =>
        {
            if (Missing())
            {
                Build();
            }
        };
    }

    static void AfterPlay(PlayModeStateChange change)
    {
        if (change == PlayModeStateChange.EnteredEditMode)
        {
            EditorApplication.playModeStateChanged -= AfterPlay;
            if (Missing())
            {
                Build();
            }
        }
    }

    static bool Missing()
    {
        foreach (var name in Names)
        {
            if (!File.Exists(PrefabPath(name)))
            {
                return true;
            }
        }
        return false;
    }

    static string PrefabPath(string name) => PrefabFolder + "/" + name + ".prefab";

    [MenuItem("Tools/Warehouse Movers/Rebuild Prefabs")]
    public static void Build()
    {
        EnsureFolder(PrefabFolder);
        EnsureFolder(MaterialFolder);
        s_Materials.Clear();
        var scene = EditorSceneManager.NewPreviewScene();
        try
        {
            Save(scene, "Worker", BuildWorker);
            Save(scene, "WorkerWithCart", BuildWorkerWithCart);
            Save(scene, "PalletJack", BuildPalletJack);
            Save(scene, "Forklift", BuildForklift);
            Save(scene, "AMR", BuildAmr);
        }
        finally
        {
            EditorSceneManager.ClosePreviewScene(scene);
        }
        AssetDatabase.SaveAssets();
        Debug.Log("WarehouseMoverBuilder: built " + string.Join(", ", Names) + " in " + PrefabFolder);
    }

    static void Save(Scene scene, string name, System.Action<Transform, WarehouseMover> build)
    {
        var root = new GameObject(name);
        SceneManager.MoveGameObjectToScene(root, scene);
        // Dynamic, upright at a fixed height (WarehouseMover drives it by velocity changes).
        var body = root.AddComponent<Rigidbody>();
        body.isKinematic = false;
        body.useGravity = false;
        body.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotationX
            | RigidbodyConstraints.FreezeRotationZ;
        body.interpolation = RigidbodyInterpolation.Interpolate;
        body.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
        var mover = root.AddComponent<WarehouseMover>();
        mover.Behaviour = WarehouseMover.Mode.Roam;
        build(root.transform, mover);
        Recentre(root.transform, mover);
        var frictionless = AssetDatabase.LoadAssetAtPath<PhysicMaterial>(FrictionlessPath);
        foreach (var collider in root.GetComponentsInChildren<Collider>())
        {
            collider.sharedMaterial = frictionless;
        }
        PrefabUtility.SaveAsPrefabAsset(root, PrefabPath(name));
    }

    // Moves the parts so the pivot is at the centre of the footprint, and records its size.
    static void Recentre(Transform root, WarehouseMover mover)
    {
        var renderers = root.GetComponentsInChildren<Renderer>();
        var bounds = renderers[0].bounds;
        foreach (var renderer in renderers)
        {
            bounds.Encapsulate(renderer.bounds);
        }
        var shift = new Vector3(-bounds.center.x, 0f, -bounds.center.z);
        foreach (Transform child in root)
        {
            child.localPosition += shift;
        }
        mover.Footprint = new Vector3(Round(bounds.size.x), Round(bounds.max.y), Round(bounds.size.z));
    }

    static float Round(float metres) => Mathf.Round(metres * 100f) / 100f;

    // ---- Models -------------------------------------------------------------------------------

    static void BuildWorker(Transform root, WarehouseMover mover)
    {
        var limbs = Person(root, "Worker", Vector3.zero, Mat("Vest", new Color(1f, 0.42f, 0.05f)));
        var walk = root.gameObject.AddComponent<WalkCycle>();
        walk.LeftLeg = limbs[0];
        walk.RightLeg = limbs[1];
        walk.LeftArm = limbs[2];
        walk.RightArm = limbs[3];
        root.GetComponent<Rigidbody>().mass = 80f;
        mover.Speed = 1.0f;
        mover.Acceleration = 0.8f;
        mover.TurnRate = 180f;
        mover.StopsPerMinute = 2f;
        mover.StopSeconds = 2.5f;
        mover.NavRadius = 0.35f;
        mover.AvoidancePriority = 60;
        mover.Gap = 0.6f;
    }

    static void BuildWorkerWithCart(Transform root, WarehouseMover mover)
    {
        var cart = Node(root, "PlatformCart", new Vector3(0f, 0f, 0.45f));
        var blue = Mat("CartBlue", new Color(0.12f, 0.28f, 0.6f), 0.4f);
        var grey = Mat("SteelGrey", new Color(0.45f, 0.47f, 0.5f), 0.5f);
        var rubber = Mat("Rubber", new Color(0.05f, 0.05f, 0.05f), 0.1f);
        Part(cart, "Deck", Shape.Box, new Vector3(0f, 0.15f, 0f), new Vector3(0.6f, 0.1f, 1f), blue);
        foreach (var x in new[] { -0.24f, 0.24f })
        {
            foreach (var z in new[] { -0.42f, 0.42f })
            {
                Part(cart, "Caster", Shape.Cylinder, new Vector3(x, 0.05f, z), new Vector3(0.1f, 0.04f, 0.1f),
                    rubber, false, new Vector3(0f, 0f, 90f));
            }
            Part(cart, "HandlePost", Shape.Box, new Vector3(x * 1.12f, 0.6f, -0.48f), new Vector3(0.03f, 0.8f, 0.03f), grey);
        }
        Part(cart, "HandleBar", Shape.Cylinder, new Vector3(0f, 1f, -0.48f), new Vector3(0.03f, 0.6f, 0.03f),
            grey, false, new Vector3(0f, 0f, 90f));
        Cartons(cart, new Vector3(0f, 0.2f, 0f), 2, 2, 2, new Vector3(0.28f, 0.25f, 0.45f));

        // The worker pushes from behind, both hands on the bar.
        var limbs = Person(root, "Worker", new Vector3(0f, 0f, -0.45f), Mat("Vest", new Color(1f, 0.42f, 0.05f)));
        limbs[2].localEulerAngles = new Vector3(-42f, 0f, 0f);
        limbs[3].localEulerAngles = new Vector3(-42f, 0f, 0f);
        var walk = root.gameObject.AddComponent<WalkCycle>();
        walk.LeftLeg = limbs[0];
        walk.RightLeg = limbs[1];
        root.GetComponent<Rigidbody>().mass = 140f;
        mover.Speed = 0.8f;
        mover.Acceleration = 0.5f;
        mover.TurnRate = 90f;
        mover.ReverseOnReturn = true;     // on a back-and-forth route: pulls the cart back
        mover.StopsPerMinute = 1f;
        mover.StopSeconds = 3f;
        mover.NavRadius = 0.5f;
        mover.AvoidancePriority = 55;
        mover.Gap = 0.7f;
    }

    static void BuildPalletJack(Transform root, WarehouseMover mover)
    {
        var red = Mat("JackRed", new Color(0.75f, 0.1f, 0.08f), 0.4f);
        var grey = Mat("SteelGrey", new Color(0.45f, 0.47f, 0.5f), 0.5f);
        var rubber = Mat("Rubber", new Color(0.05f, 0.05f, 0.05f), 0.1f);
        var jack = Node(root, "PalletJack", Vector3.zero);
        foreach (var x in new[] { -0.2f, 0.2f })
        {
            Part(jack, "Fork", Shape.Box, new Vector3(x, 0.05f, -0.02f), new Vector3(0.16f, 0.06f, 1.15f), red);
        }
        Part(jack, "PumpUnit", Shape.Box, new Vector3(0f, 0.2f, 0.72f), new Vector3(0.55f, 0.3f, 0.22f), red);
        Part(jack, "SteerWheel", Shape.Cylinder, new Vector3(0f, 0.09f, 0.72f), new Vector3(0.18f, 0.06f, 0.18f),
            rubber, false, new Vector3(0f, 0f, 90f));
        Part(jack, "Hydraulics", Shape.Cylinder, new Vector3(0f, 0.45f, 0.72f), new Vector3(0.08f, 0.25f, 0.08f), grey, false);
        var grip = new Vector3(0.12f, 0.96f, 1.09f);
        Rod(jack, "Handle", new Vector3(0f, 0.45f, 0.8f), grip, 0.04f, red, false);
        Part(jack, "Grip", Shape.Cylinder, grip, new Vector3(0.04f, 0.3f, 0.04f), rubber, false, new Vector3(0f, 0f, 90f));

        Pallet(root, new Vector3(0f, 0.085f, 0f));
        Cartons(root, new Vector3(0f, 0.229f, 0f), 2, 3, 2, new Vector3(0.38f, 0.3f, 0.38f));

        // The operator walks ahead, pulling the handle with the right hand.
        var limbs = Person(root, "Operator", new Vector3(0f, 0f, 1.45f), Mat("VestYellow", new Color(0.85f, 0.95f, 0.1f)));
        limbs[3].localEulerAngles = new Vector3(35f, 0f, 0f);
        var walk = root.gameObject.AddComponent<WalkCycle>();
        walk.LeftLeg = limbs[0];
        walk.RightLeg = limbs[1];
        walk.LeftArm = limbs[2];
        root.GetComponent<Rigidbody>().mass = 500f;
        mover.Speed = 0.8f;
        mover.Acceleration = 0.4f;
        mover.TurnRate = 60f;
        mover.ReverseOnReturn = true;     // on a back-and-forth route: pushed back the way it came
        mover.PauseSeconds = 2f;
        mover.NavRadius = 0.55f;
        mover.AvoidancePriority = 45;
        mover.Gap = 0.8f;
    }

    static void BuildForklift(Transform root, WarehouseMover mover)
    {
        var yellow = Mat("ForkliftYellow", new Color(0.95f, 0.72f, 0.08f), 0.5f);
        var dark = Mat("DarkSteel", new Color(0.13f, 0.13f, 0.14f), 0.4f);
        var rubber = Mat("Rubber", new Color(0.05f, 0.05f, 0.05f), 0.1f);
        var truck = Node(root, "Truck", Vector3.zero);
        Part(truck, "Chassis", Shape.Box, new Vector3(0f, 0.53f, 0f), new Vector3(1f, 0.9f, 1.8f), yellow);
        Part(truck, "Counterweight", Shape.Box, new Vector3(0f, 1.2f, -0.65f), new Vector3(1f, 0.5f, 0.5f), dark);
        Part(truck, "Dash", Shape.Box, new Vector3(0f, 1.13f, 0.55f), new Vector3(0.9f, 0.3f, 0.25f), yellow);
        Part(truck, "Seat", Shape.Box, new Vector3(0f, 1.03f, -0.2f), new Vector3(0.45f, 0.1f, 0.45f), rubber, false);
        Part(truck, "Backrest", Shape.Box, new Vector3(0f, 1.3f, -0.42f), new Vector3(0.45f, 0.45f, 0.08f), rubber, false);
        Rod(truck, "SteeringColumn", new Vector3(0f, 1.15f, 0.5f), new Vector3(0f, 1.4f, 0.32f), 0.05f, dark, false);
        Part(truck, "SteeringWheel", Shape.Cylinder, new Vector3(0f, 1.41f, 0.31f), new Vector3(0.34f, 0.03f, 0.34f),
            rubber, false, new Vector3(-35f, 0f, 0f));
        foreach (var x in new[] { -0.47f, 0.47f })
        {
            Part(truck, "FrontWheel", Shape.Cylinder, new Vector3(x, 0.23f, 0.6f), new Vector3(0.46f, 0.2f, 0.46f),
                rubber, false, new Vector3(0f, 0f, 90f));
            Part(truck, "RearWheel", Shape.Cylinder, new Vector3(x, 0.19f, -0.6f), new Vector3(0.38f, 0.16f, 0.38f),
                rubber, false, new Vector3(0f, 0f, 90f));
            foreach (var z in new[] { -0.5f, 0.45f })
            {
                Part(truck, "GuardPost", Shape.Box, new Vector3(x, 1.55f, z), new Vector3(0.06f, 1.15f, 0.06f), dark);
            }
            Part(truck, "GuardRail", Shape.Box, new Vector3(x, 2.15f, -0.025f), new Vector3(0.06f, 0.06f, 1.01f), dark);
        }
        for (int i = 0; i < 5; i++)
        {
            Part(truck, "GuardSlat", Shape.Box, new Vector3(0f, 2.15f, -0.5f + i * 0.2375f),
                new Vector3(0.94f, 0.04f, 0.06f), dark, false);
        }
        Part(truck, "Beacon", Shape.Cylinder, new Vector3(0.35f, 2.21f, -0.45f), new Vector3(0.1f, 0.08f, 0.1f),
            Mat("BeaconAmber", new Color(1f, 0.55f, 0f), 0.6f, true), false);

        // Mast, carriage and forks, carried at travel height (blades 0.12-0.165 m: in the lidar plane).
        var mast = Node(root, "Mast", Vector3.zero);
        foreach (var x in new[] { -0.33f, 0.33f })
        {
            Part(mast, "Upright", Shape.Box, new Vector3(x, 1.2f, 0.98f), new Vector3(0.1f, 2.3f, 0.12f), dark);
        }
        foreach (var y in new[] { 0.6f, 2.3f })
        {
            Part(mast, "CrossBar", Shape.Box, new Vector3(0f, y, 0.98f), new Vector3(0.76f, 0.08f, 0.1f), dark, false);
        }
        Part(mast, "Carriage", Shape.Box, new Vector3(0f, 0.4f, 1.08f), new Vector3(0.9f, 0.45f, 0.05f), dark);
        Part(mast, "LoadBackrest", Shape.Box, new Vector3(0f, 0.9f, 1.1f), new Vector3(0.9f, 0.6f, 0.03f), dark, false);
        foreach (var x in new[] { -0.25f, 0.25f })
        {
            Part(mast, "ForkShank", Shape.Box, new Vector3(x, 0.39f, 1.12f), new Vector3(0.12f, 0.5f, 0.045f), dark);
            Part(mast, "ForkBlade", Shape.Box, new Vector3(x, 0.1425f, 1.655f), new Vector3(0.12f, 0.045f, 1.07f), dark);
        }
        Pallet(root, new Vector3(0f, 0.165f, 1.72f));
        Cartons(root, new Vector3(0f, 0.309f, 1.72f), 2, 3, 2, new Vector3(0.38f, 0.3f, 0.38f));

        SeatedDriver(root, new Vector3(0f, 1.08f, -0.2f));
        root.GetComponent<Rigidbody>().mass = 3500f;
        mover.Speed = 1.2f;
        mover.Acceleration = 0.8f;
        mover.TurnRate = 50f;
        mover.ReverseOnReturn = true;     // on a back-and-forth route: backs out rather than turning
        mover.PauseSeconds = 1.5f;
        mover.NavRadius = 0.75f;
        mover.AvoidancePriority = 20;     // others make way for a forklift
        mover.Gap = 1.2f;
    }

    static void BuildAmr(Transform root, WarehouseMover mover)
    {
        var shell = Mat("AmrWhite", new Color(0.85f, 0.86f, 0.88f), 0.6f);
        var dark = Mat("DarkSteel", new Color(0.13f, 0.13f, 0.14f), 0.4f);
        var rubber = Mat("Rubber", new Color(0.05f, 0.05f, 0.05f), 0.1f);
        var amr = Node(root, "AMR", Vector3.zero);
        Part(amr, "Body", Shape.Box, new Vector3(0f, 0.18f, 0f), new Vector3(0.6f, 0.28f, 0.8f), shell);
        Part(amr, "Bumper", Shape.Box, new Vector3(0f, 0.1f, 0f), new Vector3(0.62f, 0.06f, 0.82f), dark, false);
        Part(amr, "StatusLight", Shape.Box, new Vector3(0f, 0.27f, 0f), new Vector3(0.61f, 0.025f, 0.81f),
            Mat("StatusCyan", new Color(0.1f, 0.85f, 1f), 0.6f, true), false);
        Part(amr, "TopPlate", Shape.Box, new Vector3(0f, 0.335f, 0f), new Vector3(0.56f, 0.03f, 0.76f), dark);
        Part(amr, "Lidar", Shape.Cylinder, new Vector3(0f, 0.345f, 0.33f), new Vector3(0.08f, 0.05f, 0.08f), rubber, false);
        foreach (var x in new[] { -0.29f, 0.29f })
        {
            Part(amr, "Wheel", Shape.Cylinder, new Vector3(x, 0.07f, 0f), new Vector3(0.14f, 0.04f, 0.14f),
                rubber, false, new Vector3(0f, 0f, 90f));
        }
        Part(amr, "Tote", Shape.Box, new Vector3(0f, 0.475f, 0f), new Vector3(0.4f, 0.25f, 0.6f),
            Mat("ToteBlue", new Color(0.1f, 0.33f, 0.75f), 0.5f));
        root.GetComponent<Rigidbody>().mass = 200f;
        mover.Speed = 1.0f;
        mover.Acceleration = 0.7f;
        mover.TurnRate = 90f;
        mover.NavRadius = 0.45f;
        mover.AvoidancePriority = 50;
        mover.Gap = 0.6f;
    }

    // ---- Parts --------------------------------------------------------------------------------

    // A standing person 1.78 m tall facing +z, feet at `feet`. Returns the left/right leg and
    // left/right arm pivots (hip and shoulder), which WalkCycle swings.
    static Transform[] Person(Transform parent, string name, Vector3 feet, Material vest)
    {
        var person = Node(parent, name, feet);
        var trousers = Mat("Trousers", new Color(0.12f, 0.15f, 0.3f), 0.2f);
        var shirt = Mat("Shirt", new Color(0.35f, 0.37f, 0.4f), 0.2f);
        var skin = Mat("Skin", new Color(0.82f, 0.62f, 0.48f), 0.3f);
        var boots = Mat("Rubber", new Color(0.05f, 0.05f, 0.05f), 0.1f);
        var stripe = Mat("Reflective", new Color(0.85f, 0.87f, 0.9f), 0.9f);
        var limbs = new Transform[4];
        for (int i = 0; i < 2; i++)
        {
            float x = i == 0 ? -0.1f : 0.1f;
            var hip = Node(person, i == 0 ? "LeftLeg" : "RightLeg", new Vector3(x, 0.9f, 0f));
            Part(hip, "Leg", Shape.Capsule, new Vector3(0f, -0.45f, 0f), new Vector3(0.15f, 0.9f, 0.15f), trousers);
            Part(hip, "Boot", Shape.Box, new Vector3(0f, -0.865f, 0.04f), new Vector3(0.11f, 0.07f, 0.26f), boots, false);
            limbs[i] = hip;

            var shoulder = Node(person, i == 0 ? "LeftArm" : "RightArm", new Vector3(x * 2.5f, 1.47f, 0f));
            Part(shoulder, "Arm", Shape.Capsule, new Vector3(0f, -0.3f, 0f), new Vector3(0.1f, 0.62f, 0.1f), shirt);
            Part(shoulder, "Hand", Shape.Sphere, new Vector3(0f, -0.62f, 0f), Vector3.one * 0.09f, skin, false);
            limbs[2 + i] = shoulder;
        }
        Part(person, "Pelvis", Shape.Box, new Vector3(0f, 0.92f, 0f), new Vector3(0.36f, 0.18f, 0.22f), trousers);
        Part(person, "Torso", Shape.Box, new Vector3(0f, 1.26f, 0f), new Vector3(0.4f, 0.52f, 0.24f), vest);
        foreach (var y in new[] { 1.12f, 1.3f })
        {
            Part(person, "VestStripe", Shape.Box, new Vector3(0f, y, 0f), new Vector3(0.41f, 0.04f, 0.25f), stripe, false);
        }
        Head(person, 1.55f, skin);
        return limbs;
    }

    // A forklift driver, seat surface at `seat`. Trim only: the truck's colliders cover it.
    static void SeatedDriver(Transform parent, Vector3 seat)
    {
        var person = Node(parent, "Driver", seat);
        var trousers = Mat("Trousers", new Color(0.12f, 0.15f, 0.3f), 0.2f);
        var shirt = Mat("Shirt", new Color(0.35f, 0.37f, 0.4f), 0.2f);
        var skin = Mat("Skin", new Color(0.82f, 0.62f, 0.48f), 0.3f);
        var vest = Mat("VestYellow", new Color(0.85f, 0.95f, 0.1f));
        foreach (var x in new[] { -0.1f, 0.1f })
        {
            Part(person, "Thigh", Shape.Capsule, new Vector3(x, 0.08f, 0.2f), new Vector3(0.15f, 0.5f, 0.15f),
                trousers, false, new Vector3(90f, 0f, 0f));
            Part(person, "Shin", Shape.Capsule, new Vector3(x, -0.13f, 0.42f), new Vector3(0.13f, 0.4f, 0.13f), trousers, false);
            // Shoulder to the steering wheel rim: down and forward.
            Part(person, "Arm", Shape.Capsule, new Vector3(x * 2.2f, 0.48f, 0.13f), new Vector3(0.1f, 0.5f, 0.1f),
                shirt, false, new Vector3(-50f, 0f, 0f));
        }
        Part(person, "Pelvis", Shape.Box, new Vector3(0f, 0.09f, -0.02f), new Vector3(0.36f, 0.18f, 0.22f), trousers, false);
        Part(person, "Torso", Shape.Box, new Vector3(0f, 0.42f, -0.05f), new Vector3(0.4f, 0.52f, 0.24f), vest, false);
        Head(person, 0.71f, skin, false);
    }

    static void Head(Transform person, float neckY, Material skin, bool solid = true)
    {
        var helmet = Mat("HardHat", new Color(1f, 0.85f, 0.05f), 0.6f);
        Part(person, "Neck", Shape.Cylinder, new Vector3(0f, neckY, 0f), new Vector3(0.1f, 0.08f, 0.1f), skin, false);
        Part(person, "Head", Shape.Sphere, new Vector3(0f, neckY + 0.09f, 0f), Vector3.one * 0.22f, skin, solid);
        Part(person, "HardHat", Shape.Sphere, new Vector3(0f, neckY + 0.15f, 0f), new Vector3(0.25f, 0.16f, 0.28f), helmet, false);
        Part(person, "HatBrim", Shape.Cylinder, new Vector3(0f, neckY + 0.13f, 0.01f), new Vector3(0.3f, 0.015f, 0.32f), helmet, false);
    }

    // A 0.8 x 1.2 x 0.144 m block pallet, long side along z, bottom at `bottom`. One box collider
    // (the sides look closed to a 2D lidar at most angles); the boards and blocks are trim.
    static void Pallet(Transform parent, Vector3 bottom)
    {
        var wood = Mat("PalletWood", new Color(0.72f, 0.57f, 0.38f), 0.15f);
        var pallet = Node(parent, "Pallet", bottom);
        var collider = pallet.gameObject.AddComponent<BoxCollider>();
        collider.center = new Vector3(0f, 0.072f, 0f);
        collider.size = new Vector3(0.8f, 0.144f, 1.2f);
        foreach (var x in new[] { -0.35f, -0.175f, 0f, 0.175f, 0.35f })
        {
            Part(pallet, "TopBoard", Shape.Box, new Vector3(x, 0.133f, 0f), new Vector3(0.1f, 0.022f, 1.2f), wood, false);
        }
        foreach (var x in new[] { -0.35f, 0f, 0.35f })
        {
            Part(pallet, "BottomBoard", Shape.Box, new Vector3(x, 0.011f, 0f), new Vector3(0.1f, 0.022f, 1.2f), wood, false);
            foreach (var z in new[] { -0.55f, 0f, 0.55f })
            {
                Part(pallet, "Block", Shape.Box, new Vector3(x, 0.072f, z), new Vector3(0.1f, 0.1f, 0.1f), wood, false);
            }
        }
    }

    // A columns x rows x layers stack of cartons centred on `bottom`.
    static void Cartons(Transform parent, Vector3 bottom, int columns, int rows, int layers, Vector3 size)
    {
        var carton = Mat("Carton", new Color(0.66f, 0.49f, 0.3f), 0.1f);
        var tape = Mat("Tape", new Color(0.78f, 0.66f, 0.45f), 0.5f);
        var stack = Node(parent, "Load", bottom);
        for (int layer = 0; layer < layers; layer++)
        {
            for (int c = 0; c < columns; c++)
            {
                for (int r = 0; r < rows; r++)
                {
                    var centre = new Vector3((c - (columns - 1) / 2f) * size.x, (layer + 0.5f) * size.y,
                        (r - (rows - 1) / 2f) * size.z);
                    var box = Part(stack, "Carton", Shape.Box, centre, size * 0.98f, carton).transform;
                    Part(box, "Tape", Shape.Box, new Vector3(0f, 0.5f, 0f), new Vector3(0.15f, 0.02f, 1.01f), tape, false);
                }
            }
        }
    }

    // ---- Helpers ------------------------------------------------------------------------------

    static Transform Node(Transform parent, string name, Vector3 position)
    {
        var node = new GameObject(name).transform;
        node.SetParent(parent, false);
        node.localPosition = position;
        return node;
    }

    // A primitive with the given real size (cylinders and capsules: diameter x height x diameter).
    // `solid` keeps a collider; a cylinder gets a convex mesh collider, since its default capsule
    // collider would round off a wheel or post.
    static GameObject Part(Transform parent, string name, Shape shape, Vector3 centre, Vector3 size,
        Material material, bool solid = true, Vector3 euler = default)
    {
        var type = shape == Shape.Box ? PrimitiveType.Cube : shape == Shape.Cylinder ? PrimitiveType.Cylinder
            : shape == Shape.Capsule ? PrimitiveType.Capsule : PrimitiveType.Sphere;
        var part = GameObject.CreatePrimitive(type);
        part.name = name;
        part.transform.SetParent(parent, false);
        part.transform.localPosition = centre;
        part.transform.localEulerAngles = euler;
        bool tall = shape == Shape.Cylinder || shape == Shape.Capsule;
        part.transform.localScale = tall ? new Vector3(size.x, size.y / 2f, size.z) : size;
        part.GetComponent<MeshRenderer>().sharedMaterial = material;
        var collider = part.GetComponent<Collider>();
        if (!solid || shape == Shape.Cylinder)
        {
            Object.DestroyImmediate(collider);
        }
        if (solid && shape == Shape.Cylinder)
        {
            part.AddComponent<MeshCollider>().convex = true;
        }
        return part;
    }

    // A cylinder from `a` to `b`.
    static void Rod(Transform parent, string name, Vector3 a, Vector3 b, float diameter, Material material, bool solid)
    {
        var part = Part(parent, name, Shape.Cylinder, (a + b) / 2f,
            new Vector3(diameter, Vector3.Distance(a, b), diameter), material, solid);
        part.transform.localRotation = Quaternion.FromToRotation(Vector3.up, b - a);
    }

    static Material Mat(string name, Color color, float smoothness = 0.25f, bool emissive = false)
    {
        if (s_Materials.TryGetValue(name, out var cached))
        {
            return cached;
        }
        var path = MaterialFolder + "/" + name + ".mat";
        var shader = Shader.Find("Universal Render Pipeline/Lit");
        bool urp = shader != null;
        if (!urp)
        {
            shader = Shader.Find("Standard");
        }
        var material = AssetDatabase.LoadAssetAtPath<Material>(path);
        bool created = material == null;
        if (created)
        {
            material = new Material(shader);
        }
        material.shader = shader;
        material.SetColor(urp ? "_BaseColor" : "_Color", color);
        material.SetFloat(urp ? "_Smoothness" : "_Glossiness", smoothness);
        if (emissive)
        {
            material.EnableKeyword("_EMISSION");
            material.SetColor("_EmissionColor", color * 2f);
            material.globalIlluminationFlags = MaterialGlobalIlluminationFlags.RealtimeEmissive;
        }
        if (created)
        {
            AssetDatabase.CreateAsset(material, path);
        }
        else
        {
            EditorUtility.SetDirty(material);
        }
        s_Materials[name] = material;
        return material;
    }

    static void EnsureFolder(string path)
    {
        if (AssetDatabase.IsValidFolder(path))
        {
            return;
        }
        var parent = Path.GetDirectoryName(path).Replace('\\', '/');
        EnsureFolder(parent);
        AssetDatabase.CreateFolder(parent, Path.GetFileName(path));
    }
}
