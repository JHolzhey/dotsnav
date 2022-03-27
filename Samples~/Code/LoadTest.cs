using DotsNav.Core.Hybrid;
using DotsNav.Data;
using DotsNav.Drawing;
using DotsNav.Hybrid;
using DotsNav.Navmesh.Hybrid;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using Random = Unity.Mathematics.Random;

namespace DotsNav.Samples.Code
{
    class LoadTest : MonoBehaviour
    {
        public Vector2 Size;
        public int Amount;
        public DotsNavObstacle[] Prefabs;
        public int Seed;
        public DotsNavPlane PlanePrefab;
        public float ScaleOffset;
        public Text Output;
        public RectTransform Help;

        DotsNavNavmesh _navmesh;
        DrawMode _drawMode = DrawMode.Constrained;
        NativeList<float2> _vertices;
        NativeList<int2> _startEnds;
        bool _shouldInsert;
        bool _delayed;
        DotsNavPlane _plane;

        void Start()
        {
            // Ensure gameobject conversion when loading a scene
            World.All[0].GetOrCreateSystem<InitializationSystemGroup>().Update();

            Help.gameObject.SetActive(!Application.isEditor);
            FindObjectOfType<CameraController>().Initialize(Size);
            _vertices = new NativeList<float2>(Allocator.Persistent);
            _startEnds = new NativeList<int2>(Allocator.Persistent);

            foreach (var prefab in Prefabs)
            {
                var start = _vertices.Length;
                foreach (var vertex in prefab.Vertices)
                    _vertices.Add(vertex);
                _startEnds.Add(new int2(start, _vertices.Length));
            }

            Load();
        }

        // Navmeshes can be created by instatiating prefabs. Obstacles can then be added by
        // instantiating prefabs. Both can later be disposed of by destroying the created
        // gameobjects. Static obstacles however can be inserted in bulk by providing a Burst
        // compatible obstacle adder as seen below.
        void Load()
        {
            _plane = Instantiate(PlanePrefab);
            _plane.GetComponent<DotsNavPlane>().Size = Size;
            _navmesh = _plane.GetComponent<DotsNavNavmesh>();
            _navmesh.DrawMode = _drawMode;
            _shouldInsert = true;
        }

        void Reload()
        {
            Seed++;
            DestroyImmediate(_plane.gameObject);
            Load();
        }

        struct ObstacleAdder : IObstacleAdder
        {
            readonly float _scaleOffset;
            readonly float2 _size;
            Random _r;
            NativeList<float2> _vertices;
            NativeList<int2> _startEnds;

            public ObstacleAdder(float scaleOffset, float2 size, Random r, NativeList<float2> vertices, NativeList<int2> startEnds)
            {
                _scaleOffset = scaleOffset;
                _size = size;
                _r = r;
                _vertices = vertices;
                _startEnds = startEnds;
            }

            public void Add(int index, DynamicBuffer<VertexElement> points)
            {
                var s = points.Length;
                var startEnd = _startEnds[_r.NextInt(_startEnds.Length)];
                var scale = 1 - _scaleOffset + _r.NextFloat() * 2 * _scaleOffset;
                var rot = _r.NextFloat(2 * math.PI);

                for (int i = startEnd.x; i < startEnd.y; i++)
                    points.Add(Rotate(scale * _vertices[i], rot));

                var min = new float2(float.MaxValue);
                var max = new float2(float.MinValue);
                for (var i = s; i < points.Length; i++)
                {
                    min = math.min(points[i], min);
                    max = math.max(points[i], max);
                }

                var size = max - min;
                var range = _size - size;
                var offset = _r.NextFloat2(range);
                for (var i = s; i < points.Length; i++)
                    points[i] += offset - min - _size / 2;
            }

            static float2 Rotate(Vector2 v, double angleRadians)
            {
                var s = math.sin(-angleRadians);
                var c = math.cos(-angleRadians);
                return (float2) new double2(v.x * c - v.y * s, v.x * s + v.y * c);
            }
        }

        void Update()
        {
            if (_shouldInsert)
            {
                if (!_delayed)
                {
                    _delayed = true;
                }
                else
                {
                    var r = new Random((uint) Seed);
                    _plane.InsertObstacleBulk(Amount, new ObstacleAdder(ScaleOffset, Size, r, _vertices, _startEnds));
                    Output.text = $"Loaded {Amount} obstacles and {_startEnds[_startEnds.Length - 1].y} vertices\nPress R to reload";
                    _shouldInsert = false;
                    _delayed = false;
                }
            }

            if (Input.GetKeyDown(KeyCode.M))
            {
                SceneManager.LoadScene("menu");
                return;
            }

            if (Input.GetKeyDown(KeyCode.R))
            {
                Reload();
                return;
            }

            if (Input.GetKeyDown(KeyCode.E))
            {
                _drawMode = _drawMode == DrawMode.Constrained ? DrawMode.Both : DrawMode.Constrained;
                _navmesh.DrawMode = _drawMode;
            }

            if (Input.GetKeyDown(KeyCode.H))
                Help.gameObject.SetActive(!Help.gameObject.activeSelf);
        }

        void OnDestroy()
        {
            _vertices.Dispose();
            _startEnds.Dispose();
        }
    }
}
