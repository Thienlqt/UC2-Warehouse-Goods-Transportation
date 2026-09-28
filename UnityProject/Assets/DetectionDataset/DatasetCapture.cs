using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using RosSharp.Control;
using UnityEngine;
using UnityEngine.Perception.GroundTruth;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

namespace Unity.Robotics.Nav2Example.DetectionDataset
{
    /// <summary>
    /// Captures a YOLO detection dataset from the warehouse scene through a camera that matches the
    /// robot's RosCameraSensor (resolution, vertical FOV, mount height, URP camera settings).
    ///
    /// Classes come from the warehouse's Perception Labeling components (box, shelf, station). Each
    /// frame renders RGB, then an object-ID pass: every labelled instance drawn unlit in its own
    /// colour at render scale 1 without MSAA. An instance's box is the extent of its visible ID
    /// pixels, so occluded parts are excluded. Every FramesPerLayout frames the layout is
    /// re-randomised: extra floor boxes, some scene boxes hidden, light intensity and tint.
    /// Train/val is split by layout so validation frames never share a layout with training.
    ///
    /// Start it from the menu Robotics > Detection Dataset, or from a script by writing key=value
    /// lines (frames, out, seed, val, per_layout) to Temp/detection_capture.request in the project.
    /// </summary>
    public class DatasetCapture : MonoBehaviour
    {
        public static readonly string[] Classes = { "box", "shelf", "station" };

        public int Frames = 9000;
        public float ValFraction = 0.1f;
        // Default: <repo>/detection_training/dataset. Its images/ and labels/ are replaced.
        public string OutputDir;
        public int Seed = 1;
        public int FramesPerLayout = 10;
        // Camera distance from the target object's surface (m); the far end matters for early warning.
        public Vector2 DistanceRange = new Vector2(0.3f, 14f);
        // Share of frames aimed at a labelled object; the rest look in a random direction.
        [Range(0, 1)]
        public float TargetedFraction = 0.8f;
        public int MinPixels = 20;
        public int MinBoxSide = 3;
        // Drop an instance whose visible box covers less than this share of its projected bounds.
        public float MinVisibleFraction = 0.12f;
        public int JpegQuality = 85;

        int m_Width = 640;
        int m_Height = 480;
        float m_VerticalFov = 42f;
        float m_HorizontalFov;
        float m_MountHeight = 0.17f;
        float m_NearClip = 0.03f;

        Transform m_Rig;
        Camera m_Rgb;
        Camera m_Id;
        RenderTexture m_RgbTarget;
        RenderTexture m_IdTarget;
        Texture2D m_RgbTex;
        Texture2D m_IdTex;
        Material m_IdMaterial;
        MaterialPropertyBlock m_Block;
        Bounds m_Floor;

        // Per layout: renderers with their ID colour (0 = background), instances and their classes.
        readonly List<Renderer> m_Renderers = new List<Renderer>();
        readonly List<uint> m_RendererKeys = new List<uint>();
        readonly List<Material[]> m_OriginalMaterials = new List<Material[]>();
        readonly List<Material[]> m_IdMaterials = new List<Material[]>();
        readonly List<bool> m_HideInIdPass = new List<bool>();
        readonly Dictionary<uint, int> m_KeyToInstance = new Dictionary<uint, int>();
        readonly List<int> m_InstanceClass = new List<int>();
        readonly List<Bounds> m_InstanceBounds = new List<Bounds>();

        readonly List<GameObject> m_SceneBoxes = new List<GameObject>();
        readonly List<GameObject> m_Spawned = new List<GameObject>();
        readonly List<(Light light, float intensity, Color colour)> m_Lights = new List<(Light, float, Color)>();
        float m_AmbientIntensity;

        readonly int[] m_ClassCounts = new int[Classes.Length];
        int m_FramesWithLabels;

        public void Apply(string args)
        {
            foreach (var line in args.Split('\n', ';'))
            {
                var kv = line.Split(new[] { '=' }, 2);
                if (kv.Length != 2)
                {
                    continue;
                }
                var value = kv[1].Trim();
                switch (kv[0].Trim())
                {
                    case "frames": Frames = int.Parse(value); break;
                    case "out": OutputDir = value; break;
                    case "seed": Seed = int.Parse(value); break;
                    case "val": ValFraction = float.Parse(value, CultureInfo.InvariantCulture); break;
                    case "per_layout": FramesPerLayout = int.Parse(value); break;
                }
            }
        }

        IEnumerator Start()
        {
            Application.targetFrameRate = -1;
            QualitySettings.vSyncCount = 0;
            Random.InitState(Seed);
            if (string.IsNullOrEmpty(OutputDir))
            {
                OutputDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "detection_training", "dataset"));
            }
            // Let the scene finish its first frame (warehouse spawners, robot setup) before touching it.
            yield return null;

            if (!Setup())
            {
                Finish();
                yield break;
            }
            PrepareOutput();
            Debug.Log(string.Format("DatasetCapture: {0} frames of {1}x{2}, vertical FOV {3}, camera height {4:0.00} m -> {5}",
                Frames, m_Width, m_Height, m_VerticalFov, m_MountHeight, OutputDir));

            var layout = -1;
            var started = Time.realtimeSinceStartup;
            for (var frame = 0; frame < Frames; frame++)
            {
                if (frame % FramesPerLayout == 0)
                {
                    layout++;
                    RandomizeLayout();
                    yield return null;  // destroyed objects leave, spawned physics settles a frame
                    BuildIdMap();
                }
                var split = IsValLayout(layout) ? "val" : "train";
                if (!SamplePose(out var pos, out var rot))
                {
                    continue;
                }
                m_Rig.SetPositionAndRotation(pos, rot);
                CaptureFrame(Path.Combine(OutputDir, "images", split, frame.ToString("D6") + ".jpg"),
                    Path.Combine(OutputDir, "labels", split, frame.ToString("D6") + ".txt"));

                if ((frame + 1) % 100 == 0 || frame + 1 == Frames)
                {
                    var elapsed = Time.realtimeSinceStartup - started;
                    var status = string.Format(CultureInfo.InvariantCulture,
                        "{0}/{1} frames, {2:0.0} fps, labelled frames {3}, instances {4}",
                        frame + 1, Frames, (frame + 1) / elapsed, m_FramesWithLabels, ClassCountsText());
                    File.WriteAllText(Path.Combine(OutputDir, "capture_status.txt"), status + "\n");
                    Debug.Log("DatasetCapture: " + status);
                }
                yield return null;
            }
            WriteMetadata();
            File.AppendAllText(Path.Combine(OutputDir, "capture_status.txt"), "done\n");
            Finish();
        }

        bool IsValLayout(int layout)
        {
            var every = Mathf.Max(2, Mathf.RoundToInt(1f / Mathf.Max(ValFraction, 0.01f)));
            return layout % every == every - 1;
        }

        bool Setup()
        {
            var sensor = FindObjectOfType<RosCameraSensor>();
            Camera template = null;
            if (sensor != null)
            {
                m_Width = sensor.Width;
                m_Height = sensor.Height;
                m_VerticalFov = sensor.VerticalFovDegrees;
                m_MountHeight = sensor.transform.position.y;
                m_NearClip = sensor.NearClipPlane;
                template = sensor.GetComponent<Camera>();
            }
            else
            {
                Debug.LogWarning("DatasetCapture: no RosCameraSensor in the scene; using 640x480, 42 deg, 0.17 m.");
            }
            var aspect = (float)m_Width / m_Height;
            m_HorizontalFov = 2f * Mathf.Atan(Mathf.Tan(m_VerticalFov * Mathf.Deg2Rad / 2f) * aspect) * Mathf.Rad2Deg;

            // The robot is not a class and would sit in front of every frame: take it out.
            foreach (var robot in FindObjectsOfType<AGVController>())
            {
                robot.gameObject.SetActive(false);
            }

            m_Rig = new GameObject("DatasetCameraRig").transform;
            m_Rig.SetParent(transform, false);
            m_RgbTarget = new RenderTexture(m_Width, m_Height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
            m_Rgb = CreateCamera("Rgb", m_RgbTarget);
            if (template != null)
            {
                m_Rgb.CopyFrom(template);
                m_Rgb.targetTexture = m_RgbTarget;
                var src = template.GetUniversalAdditionalCameraData();
                var dst = m_Rgb.GetUniversalAdditionalCameraData();
                dst.renderPostProcessing = src.renderPostProcessing;
                dst.antialiasing = src.antialiasing;
                dst.antialiasingQuality = src.antialiasingQuality;
                dst.renderShadows = src.renderShadows;
                dst.volumeLayerMask = src.volumeLayerMask;
                dst.dithering = src.dithering;
            }
            m_Rgb.transform.SetParent(m_Rig, false);
            m_Rgb.transform.localPosition = Vector3.zero;
            m_Rgb.transform.localRotation = Quaternion.identity;
            m_Rgb.fieldOfView = m_VerticalFov;
            m_Rgb.nearClipPlane = m_NearClip;
            m_Rgb.enabled = false;

            m_IdTarget = new RenderTexture(m_Width, m_Height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.Linear)
            {
                antiAliasing = 1,
                filterMode = FilterMode.Point,
            };
            m_Id = CreateCamera("Id", m_IdTarget);
            m_Id.fieldOfView = m_VerticalFov;
            m_Id.nearClipPlane = m_Rgb.nearClipPlane;
            m_Id.farClipPlane = m_Rgb.farClipPlane;
            m_Id.clearFlags = CameraClearFlags.SolidColor;
            m_Id.backgroundColor = Color.clear;
            m_Id.allowMSAA = false;
            m_Id.allowHDR = false;
            var idData = m_Id.GetUniversalAdditionalCameraData();
            idData.renderPostProcessing = false;
            idData.antialiasing = AntialiasingMode.None;
            idData.renderShadows = false;
            idData.dithering = false;
            m_Id.enabled = false;

            m_RgbTex = new Texture2D(m_Width, m_Height, TextureFormat.RGB24, false);
            m_IdTex = new Texture2D(m_Width, m_Height, TextureFormat.RGBA32, false, true);
            m_IdMaterial = new Material(Shader.Find("Universal Render Pipeline/Unlit"));
            m_Block = new MaterialPropertyBlock();

            // Walkable area: the floor tiles (Perception label "ground").
            var floor = FindObjectsOfType<Labeling>().Where(l => l.labels.Contains("ground"))
                .SelectMany(l => l.GetComponentsInChildren<Renderer>()).ToList();
            if (floor.Count == 0)
            {
                Debug.LogError("DatasetCapture: no Labeling with 'ground' found. Open SimpleWarehouseScene.");
                return false;
            }
            m_Floor = floor[0].bounds;
            foreach (var r in floor)
            {
                m_Floor.Encapsulate(r.bounds);
            }

            foreach (var labeling in FindObjectsOfType<Labeling>())
            {
                if (ClassOf(labeling) == 0)
                {
                    m_SceneBoxes.Add(labeling.gameObject);
                }
            }
            foreach (var light in FindObjectsOfType<Light>())
            {
                m_Lights.Add((light, light.intensity, light.color));
            }
            m_AmbientIntensity = RenderSettings.ambientIntensity;
            if (m_SceneBoxes.Count == 0)
            {
                Debug.LogError("DatasetCapture: no labelled boxes in the scene.");
                return false;
            }
            return true;
        }

        Camera CreateCamera(string name, RenderTexture target)
        {
            var go = new GameObject(name);
            go.transform.SetParent(m_Rig, false);
            var cam = go.AddComponent<Camera>();
            cam.targetTexture = target;
            return cam;
        }

        void PrepareOutput()
        {
            foreach (var sub in new[] { "images", "labels" })
            {
                var dir = Path.Combine(OutputDir, sub);
                if (Directory.Exists(dir))
                {
                    Directory.Delete(dir, true);
                }
                Directory.CreateDirectory(Path.Combine(dir, "train"));
                Directory.CreateDirectory(Path.Combine(dir, "val"));
            }
        }

        static int ClassOf(Labeling labeling)
        {
            foreach (var label in labeling.labels)
            {
                var index = System.Array.IndexOf(Classes, label);
                if (index >= 0)
                {
                    return index;
                }
            }
            return -1;
        }

        void RandomizeLayout()
        {
            foreach (var go in m_Spawned)
            {
                go.SetActive(false);
                Destroy(go);
            }
            m_Spawned.Clear();

            foreach (var box in m_SceneBoxes)
            {
                box.SetActive(Random.value > 0.25f);
            }

            // Extra boxes on the floor, cloned from the scene's own boxes (both prefab variants).
            Physics.SyncTransforms();
            var count = Random.Range(4, 20);
            for (var i = 0; i < count; i++)
            {
                var source = m_SceneBoxes[Random.Range(0, m_SceneBoxes.Count)];
                var clone = Instantiate(source, transform);
                clone.SetActive(true);
                clone.transform.localScale = source.transform.lossyScale;
                foreach (var body in clone.GetComponentsInChildren<Rigidbody>())
                {
                    body.isKinematic = true;
                }
                if (PlaceOnFloor(clone))
                {
                    m_Spawned.Add(clone);
                }
                else
                {
                    clone.SetActive(false);
                    Destroy(clone);
                }
            }

            foreach (var (light, intensity, colour) in m_Lights)
            {
                if (light == null)
                {
                    continue;
                }
                light.intensity = intensity * Random.Range(0.5f, 1.4f);
                var tint = Random.value < 0.5f ? new Color(1f, 0.85f, 0.7f) : new Color(0.8f, 0.9f, 1f);
                light.color = Color.Lerp(colour, tint, Random.Range(0f, 0.4f));
            }
            RenderSettings.ambientIntensity = m_AmbientIntensity * Random.Range(0.6f, 1.3f);
        }

        bool PlaceOnFloor(GameObject go)
        {
            for (var attempt = 0; attempt < 20; attempt++)
            {
                var x = Random.Range(m_Floor.min.x + 1f, m_Floor.max.x - 1f);
                var z = Random.Range(m_Floor.min.z + 1f, m_Floor.max.z - 1f);
                go.transform.SetPositionAndRotation(new Vector3(x, 0, z), Quaternion.Euler(0, Random.Range(0f, 360f), 0));
                var bounds = RendererBounds(go);
                go.transform.position += Vector3.up * (m_Floor.max.y - bounds.min.y);
                bounds = RendererBounds(go);
                Physics.SyncTransforms();
                // Shrink slightly and lift off the floor so touching the floor or a neighbour is allowed.
                var centre = bounds.center + Vector3.up * 0.05f;
                var hits = Physics.OverlapBox(centre, bounds.extents * 0.9f, Quaternion.identity, ~0,
                    QueryTriggerInteraction.Ignore);
                if (hits.All(h => h.transform.IsChildOf(go.transform)))
                {
                    return true;
                }
            }
            return false;
        }

        static Bounds RendererBounds(GameObject go)
        {
            var renderers = go.GetComponentsInChildren<Renderer>();
            var bounds = renderers.Length > 0 ? renderers[0].bounds : new Bounds(go.transform.position, Vector3.zero);
            foreach (var r in renderers)
            {
                bounds.Encapsulate(r.bounds);
            }
            return bounds;
        }

        // Instance i gets colour (i+1) * odd constant mod 2^24: unique and never black (background), and
        // spread out so a colour blended at an edge is very unlikely to equal another instance's.
        static uint KeyFor(int instance) => ((uint)(instance + 1) * 2654435761u) & 0xFFFFFFu;

        void BuildIdMap()
        {
            m_Renderers.Clear();
            m_RendererKeys.Clear();
            m_OriginalMaterials.Clear();
            m_IdMaterials.Clear();
            m_HideInIdPass.Clear();
            m_KeyToInstance.Clear();
            m_InstanceClass.Clear();
            m_InstanceBounds.Clear();
            var instanceOf = new Dictionary<Labeling, int>();

            foreach (var r in FindObjectsOfType<Renderer>())
            {
                if (!r.enabled || r.transform.IsChildOf(m_Rig))
                {
                    continue;
                }
                var labeling = r.GetComponentInParent<Labeling>();
                var cls = labeling != null ? ClassOf(labeling) : -1;
                var key = 0u;
                if (cls >= 0)
                {
                    if (!instanceOf.TryGetValue(labeling, out var instance))
                    {
                        instance = m_InstanceClass.Count;
                        instanceOf[labeling] = instance;
                        m_InstanceClass.Add(cls);
                        m_InstanceBounds.Add(r.bounds);
                        m_KeyToInstance[KeyFor(instance)] = instance;
                    }
                    else
                    {
                        var b = m_InstanceBounds[instance];
                        b.Encapsulate(r.bounds);
                        m_InstanceBounds[instance] = b;
                    }
                    key = KeyFor(instance);
                }
                var materials = r.sharedMaterials;
                // Only mesh renderers take the ID material; transparent ones (skylights) and others
                // (particles, lines) are left out of the ID pass rather than drawn as solid occluders.
                var hide = !(r is MeshRenderer || r is SkinnedMeshRenderer) ||
                    materials.Any(m => m != null && m.renderQueue >= (int)RenderQueue.Transparent);
                m_Renderers.Add(r);
                m_RendererKeys.Add(key);
                m_OriginalMaterials.Add(materials);
                m_IdMaterials.Add(Enumerable.Repeat(m_IdMaterial, Mathf.Max(1, materials.Length)).ToArray());
                m_HideInIdPass.Add(hide);
            }
        }

        bool SamplePose(out Vector3 position, out Quaternion rotation)
        {
            for (var attempt = 0; attempt < 60; attempt++)
            {
                var height = Mathf.Max(0.08f, m_MountHeight + Random.Range(-0.04f, 0.12f));
                Vector3 p;
                float yaw;
                if (Random.value < TargetedFraction && m_InstanceClass.Count > 0)
                {
                    var target = m_InstanceBounds[PickInstance()];
                    var bearing = Quaternion.Euler(0, Random.Range(0f, 360f), 0) * Vector3.forward;
                    var surface = (target.extents.x + target.extents.z) / 2f;
                    var distance = surface + Random.Range(DistanceRange.x, DistanceRange.y);
                    p = target.center - bearing * distance;
                    var look = target.center - p;
                    look.y = 0;
                    // Anywhere across the frame, not always centred.
                    yaw = Quaternion.LookRotation(look).eulerAngles.y + Random.Range(-0.45f, 0.45f) * m_HorizontalFov;
                }
                else
                {
                    p = new Vector3(Random.Range(m_Floor.min.x, m_Floor.max.x), 0, Random.Range(m_Floor.min.z, m_Floor.max.z));
                    yaw = Random.Range(0f, 360f);
                }
                p.y = m_Floor.max.y + height;
                if (!IsFree(p, height))
                {
                    continue;
                }
                position = p;
                // The robot drives level: small pitch and roll only.
                rotation = Quaternion.Euler(Random.Range(-4f, 4f), yaw, Random.Range(-2f, 2f));
                return true;
            }
            position = Vector3.zero;
            rotation = Quaternion.identity;
            return false;
        }

        // Class first (uniform over classes present), then instance, so rare classes (stations) get frames.
        int PickInstance()
        {
            var present = Enumerable.Range(0, Classes.Length).Where(c => m_InstanceClass.Contains(c)).ToList();
            var cls = present[Random.Range(0, present.Count)];
            var candidates = Enumerable.Range(0, m_InstanceClass.Count).Where(i => m_InstanceClass[i] == cls).ToList();
            return candidates[Random.Range(0, candidates.Count)];
        }

        bool IsFree(Vector3 p, float height)
        {
            var inset = 0.3f;
            if (p.x < m_Floor.min.x + inset || p.x > m_Floor.max.x - inset ||
                p.z < m_Floor.min.z + inset || p.z > m_Floor.max.z - inset)
            {
                return false;
            }
            // Where the robot body would be, and the air above it (inside a shelf, under a box).
            var radius = Mathf.Min(0.1f, height - 0.03f);
            if (Physics.CheckSphere(p, radius, ~0, QueryTriggerInteraction.Ignore))
            {
                return false;
            }
            var bottom = new Vector3(p.x, m_Floor.max.y + 0.45f, p.z);
            var top = new Vector3(p.x, m_Floor.max.y + 1.5f, p.z);
            return !Physics.CheckCapsule(bottom, top, 0.25f, ~0, QueryTriggerInteraction.Ignore);
        }

        void CaptureFrame(string imagePath, string labelPath)
        {
            m_Rgb.Render();
            RenderTexture.active = m_RgbTarget;
            m_RgbTex.ReadPixels(new Rect(0, 0, m_Width, m_Height), 0, 0, false);
            File.WriteAllBytes(imagePath, m_RgbTex.EncodeToJPG(JpegQuality));

            RenderIds();
            RenderTexture.active = m_IdTarget;
            m_IdTex.ReadPixels(new Rect(0, 0, m_Width, m_Height), 0, 0, false);
            RenderTexture.active = null;
            File.WriteAllText(labelPath, Labels());
        }

        void RenderIds()
        {
            var asset = GraphicsSettings.currentRenderPipeline as UniversalRenderPipelineAsset;
            var renderScale = asset != null ? asset.renderScale : 1f;
            var fog = RenderSettings.fog;
            if (asset != null)
            {
                asset.renderScale = 1f;  // supersampling would blend ID colours at edges
            }
            RenderSettings.fog = false;
            for (var i = 0; i < m_Renderers.Count; i++)
            {
                var r = m_Renderers[i];
                if (r == null)
                {
                    continue;
                }
                if (m_HideInIdPass[i])
                {
                    r.forceRenderingOff = true;
                    continue;
                }
                var key = m_RendererKeys[i];
                m_Block.SetVector("_BaseColor", new Vector4((key & 0xFF) / 255f, ((key >> 8) & 0xFF) / 255f, ((key >> 16) & 0xFF) / 255f, 1f));
                r.sharedMaterials = m_IdMaterials[i];
                r.SetPropertyBlock(m_Block);
            }

            m_Id.Render();

            for (var i = 0; i < m_Renderers.Count; i++)
            {
                var r = m_Renderers[i];
                if (r == null)
                {
                    continue;
                }
                if (m_HideInIdPass[i])
                {
                    r.forceRenderingOff = false;
                    continue;
                }
                r.sharedMaterials = m_OriginalMaterials[i];
                r.SetPropertyBlock(null);
            }
            RenderSettings.fog = fog;
            if (asset != null)
            {
                asset.renderScale = renderScale;
            }
        }

        string Labels()
        {
            var n = m_InstanceClass.Count;
            var minX = new int[n];
            var minY = new int[n];
            var maxX = new int[n];
            var maxY = new int[n];
            var count = new int[n];
            for (var i = 0; i < n; i++)
            {
                minX[i] = minY[i] = int.MaxValue;
                maxX[i] = maxY[i] = -1;
            }
            var pixels = m_IdTex.GetRawTextureData<Color32>();
            for (var i = 0; i < pixels.Length; i++)
            {
                var c = pixels[i];
                var key = c.r | ((uint)c.g << 8) | ((uint)c.b << 16);
                if (key == 0 || !m_KeyToInstance.TryGetValue(key, out var instance))
                {
                    continue;  // background, or a colour blended at an edge
                }
                var x = i % m_Width;
                var y = m_Height - 1 - i / m_Width;  // texture rows start at the bottom
                count[instance]++;
                if (x < minX[instance]) minX[instance] = x;
                if (x > maxX[instance]) maxX[instance] = x;
                if (y < minY[instance]) minY[instance] = y;
                if (y > maxY[instance]) maxY[instance] = y;
            }

            var text = new StringBuilder();
            for (var i = 0; i < n; i++)
            {
                var w = maxX[i] - minX[i] + 1;
                var h = maxY[i] - minY[i] + 1;
                if (count[i] < MinPixels || w < MinBoxSide || h < MinBoxSide)
                {
                    continue;
                }
                // Mostly hidden (e.g. a box seen only through pallet gaps): its visible extent is a
                // misleading sliver, so leave it out.
                var projected = ProjectedArea(m_InstanceBounds[i]);
                if (projected > 0 && w * h < MinVisibleFraction * projected)
                {
                    continue;
                }
                text.AppendFormat(CultureInfo.InvariantCulture, "{0} {1:0.000000} {2:0.000000} {3:0.000000} {4:0.000000}\n",
                    m_InstanceClass[i], (minX[i] + w / 2f) / m_Width, (minY[i] + h / 2f) / m_Height,
                    (float)w / m_Width, (float)h / m_Height);
                m_ClassCounts[m_InstanceClass[i]]++;
            }
            if (text.Length > 0)
            {
                m_FramesWithLabels++;
            }
            return text.ToString();
        }

        // Area (pixels) of the instance's world bounds projected into the image and clipped to it;
        // -1 when a corner is behind the near plane (projection meaningless, so no check).
        float ProjectedArea(Bounds b)
        {
            float minX = float.MaxValue, minY = float.MaxValue, maxX = float.MinValue, maxY = float.MinValue;
            for (var corner = 0; corner < 8; corner++)
            {
                var p = b.center + Vector3.Scale(b.extents, new Vector3(
                    (corner & 1) == 0 ? -1 : 1, (corner & 2) == 0 ? -1 : 1, (corner & 4) == 0 ? -1 : 1));
                var s = m_Rgb.WorldToScreenPoint(p);
                if (s.z < m_NearClip)
                {
                    return -1;
                }
                minX = Mathf.Min(minX, s.x);
                maxX = Mathf.Max(maxX, s.x);
                minY = Mathf.Min(minY, s.y);
                maxY = Mathf.Max(maxY, s.y);
            }
            var w = Mathf.Clamp(maxX, 0, m_Width) - Mathf.Clamp(minX, 0, m_Width);
            var h = Mathf.Clamp(maxY, 0, m_Height) - Mathf.Clamp(minY, 0, m_Height);
            return w * h;
        }

        string ClassCountsText()
        {
            return string.Join(", ", Classes.Select((c, i) => c + " " + m_ClassCounts[i]));
        }

        void WriteMetadata()
        {
            File.WriteAllText(Path.Combine(OutputDir, "classes.txt"), string.Join("\n", Classes) + "\n");
            var yaml = new StringBuilder();
            yaml.AppendLine("# Generated by DatasetCapture (Unity). Classes must match classes.txt.");
            yaml.AppendLine("path: " + OutputDir);
            yaml.AppendLine("train: images/train");
            yaml.AppendLine("val: images/val");
            yaml.AppendLine("names:");
            for (var i = 0; i < Classes.Length; i++)
            {
                yaml.AppendLine(string.Format("  {0}: {1}", i, Classes[i]));
            }
            File.WriteAllText(Path.Combine(OutputDir, "data.yaml"), yaml.ToString());
            Debug.Log("DatasetCapture: done. " + ClassCountsText());
        }

        void Finish()
        {
#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
#else
            Application.Quit();
#endif
        }

        void OnDestroy()
        {
            if (m_RgbTarget != null) m_RgbTarget.Release();
            if (m_IdTarget != null) m_IdTarget.Release();
        }
    }

#if UNITY_EDITOR
    static class DatasetCaptureLauncher
    {
        const string k_PendingArgs = "DetectionDataset.PendingArgs";
        const string k_RequestFile = "Temp/detection_capture.request";
        static double s_NextPoll;

        [UnityEditor.MenuItem("Robotics/Detection Dataset/Capture Full Dataset")]
        static void CaptureFull() => Launch("");

        [UnityEditor.MenuItem("Robotics/Detection Dataset/Capture Preview (100 frames)")]
        static void CapturePreview() => Launch("frames=100\nout=" +
            Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "detection_training", "dataset_preview")));

        static void Launch(string args)
        {
            if (UnityEditor.EditorApplication.isPlayingOrWillChangePlaymode)
            {
                Debug.LogWarning("DatasetCapture: stop Play mode first.");
                return;
            }
            // Survives the domain reload on entering Play mode; the trailing newline keeps it non-empty.
            UnityEditor.SessionState.SetString(k_PendingArgs, args + "\n");
            UnityEditor.EditorApplication.isPlaying = true;
        }

        [UnityEditor.InitializeOnLoadMethod]
        static void WatchRequestFile()
        {
            UnityEditor.EditorApplication.update += () =>
            {
                if (UnityEditor.EditorApplication.timeSinceStartup < s_NextPoll)
                {
                    return;
                }
                s_NextPoll = UnityEditor.EditorApplication.timeSinceStartup + 1.0;
                if (UnityEditor.EditorApplication.isPlayingOrWillChangePlaymode ||
                    UnityEditor.EditorApplication.isCompiling || !File.Exists(k_RequestFile))
                {
                    return;
                }
                var args = File.ReadAllText(k_RequestFile);
                File.Delete(k_RequestFile);
                Launch(args);
            };
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void StartIfPending()
        {
            var args = UnityEditor.SessionState.GetString(k_PendingArgs, "");
            if (args.Length == 0)
            {
                return;
            }
            UnityEditor.SessionState.EraseString(k_PendingArgs);
            new GameObject("DatasetCapture").AddComponent<DatasetCapture>().Apply(args);
        }
    }
#endif
}
