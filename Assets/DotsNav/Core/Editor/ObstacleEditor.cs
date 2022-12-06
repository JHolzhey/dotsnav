#if UNITY_EDITOR
using System.Linq;
using DotsNav.Hybrid;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;

namespace DotsNav
{
    [CustomEditor(typeof(DotsNavObstacle), true)]
    [CanEditMultipleObjects]
    class ObstacleEditor : Editor
    {
        const int Width = 50;
        int _vertexIndex;
        bool _initialized;
        bool _pivot;
        static Material _lineMat;

        static bool Key => Event.current.type == EventType.KeyDown;
        static bool KeyUp => Key && Event.current.keyCode == DotsNavPrefs.VertexUp;
        static bool KeyDown => Key && Event.current.keyCode == DotsNavPrefs.VertexDown;

        void OnSceneGUI()
        {
            if (Application.isPlaying || Selection.count > 1)
                return;

            if (!_initialized)
            {
                // todo intercept keys here? this breaks arrow keys in hierarchy
                // SceneView.lastActiveSceneView.Focus();
                _initialized = true;
            }

            var obstacle = (DotsNavObstacle)target;
            var cam = SceneView.lastActiveSceneView.camera;
            if (_lineMat == null)
                _lineMat = new Material(Shader.Find("Lines/Colored Blended"));
            _lineMat.SetPass(0);
            GL.PushMatrix();
            GL.LoadPixelMatrix();
            GL.Begin(GL.LINES);

            for (var i = 0; i < obstacle.Vertices.Length; i++)
            {
                var c = Color.Lerp(DotsNavPrefs.EditColor, DotsNavPrefs.FadeColor, i * (.8f / (obstacle.Vertices.Length - 1)));
                GL.Color(c);
                var vertex = obstacle.GetVertexWorldSpace(i);
                var screenPoint = (float3)cam.WorldToScreenPoint(vertex);
                DrawPoint(screenPoint.xy);
            }

            GL.End();
            GL.PopMatrix();

            Handles.BeginGUI();

            _pivot = GUILayout.Toggle(_pivot, "Pivot");
            if (_pivot)
            {
                if (GUILayout.Button("P ↔", GUILayout.Width(Width)))
                {
                    Undo.RecordObject(obstacle, "Edit obstacle");
                    var min = new float2(float.MaxValue);
                    var max = new float2(float.MinValue);
                    foreach (var vertex in obstacle.Vertices)
                    {
                        min = math.min(min, vertex);
                        max = math.max(max, vertex);
                    }

                    Vector2 p = min + (max - min) / 2;
                    for (var i = 0; i < obstacle.Vertices.Length; i++)
                        obstacle.Vertices[i] -= p;
                }
                if (GUILayout.Button("P ↙", GUILayout.Width(Width)))
                {
                    Undo.RecordObject(obstacle, "Edit obstacle");
                    Vector2 p = new float2(float.MaxValue);
                    foreach (var vertex in obstacle.Vertices)
                        p = math.min(p, vertex);
                    for (var i = 0; i < obstacle.Vertices.Length; i++)
                        obstacle.Vertices[i] -= p;
                }
                if (GUILayout.Button("P ↖", GUILayout.Width(Width)))
                {
                    Undo.RecordObject(obstacle, "Edit obstacle");
                    Vector2 p = new float2(float.MaxValue, float.MinValue);
                    foreach (var vertex in obstacle.Vertices)
                        p = new float2(math.min(p.x, vertex.x), math.max(p.y, vertex.y));
                    for (var i = 0; i < obstacle.Vertices.Length; i++)
                        obstacle.Vertices[i] -= p;
                }
                if (GUILayout.Button("P ↗", GUILayout.Width(Width)))
                {
                    Undo.RecordObject(obstacle, "Edit obstacle");
                    Vector2 p = new float2(float.MinValue);
                    foreach (var vertex in obstacle.Vertices)
                        p = math.max(p, vertex);
                    for (var i = 0; i < obstacle.Vertices.Length; i++)
                        obstacle.Vertices[i] -= p;
                }
                if (GUILayout.Button("P ↘", GUILayout.Width(Width)))
                {
                    Undo.RecordObject(obstacle, "Edit obstacle");
                    Vector2 p = new float2(float.MinValue, float.MaxValue);
                    foreach (var vertex in obstacle.Vertices)
                        p = new float2(math.max(p.x, vertex.x), math.min(p.y, vertex.y));
                    for (var i = 0; i < obstacle.Vertices.Length; i++)
                        obstacle.Vertices[i] -= p;
                }

                {
                    var min = new float3(float.MaxValue);
                    var max = new float3(float.MinValue);
                    for (var i = 0; i < obstacle.Vertices.Length; i++)
                    {
                        var vertex = obstacle.GetVertexWorldSpace(i);
                        min = math.min(min, vertex);
                        max = math.max(max, vertex);
                    }

                    Handles.EndGUI();
                    var pos = (min + max) / 2;

                    EditorGUI.BeginChangeCheck();
                    float3 newPos = Handles.PositionHandle(pos, obstacle.transform.rotation);

                    if (EditorGUI.EndChangeCheck())
                    {
                        Undo.RecordObject(obstacle, "Edit obstacle");
                        Vector2 delta = math.mul(math.inverse(obstacle.transform.rotation), newPos - pos).xz;
                        for (int i = 0; i < obstacle.Vertices.Length; i++)
                            obstacle.Vertices[i] += delta;
                    }
                }
            }
            else
            {
                GUILayout.Label($"{_vertexIndex}/{obstacle.Vertices.Length - 1}");
                if (GUILayout.Button($"↑ ({DotsNavPrefs.VertexUp})", GUILayout.Width(Width)) || KeyUp)
                    if (++_vertexIndex >= obstacle.Vertices.Length)
                        _vertexIndex = 0;

                if (GUILayout.Button($"↓ ({DotsNavPrefs.VertexDown})", GUILayout.Width(Width)) || KeyDown)
                    if (--_vertexIndex == -1)
                        _vertexIndex = math.max(0, obstacle.Vertices.Length - 1);

                if (GUILayout.Button("+ ↑", GUILayout.Width(Width))) CreateUp();
                if (GUILayout.Button("+ ↓", GUILayout.Width(Width))) CreateDown();
                if (GUILayout.Button(" X", GUILayout.Width(Width)) && obstacle.Vertices.Length > 2) Remove();
                Handles.EndGUI();

                EditorGUI.BeginChangeCheck();
                var pos = obstacle.GetVertexWorldSpace(_vertexIndex);
                float3 newPos = Handles.PositionHandle(pos, obstacle.transform.rotation);
                if (EditorGUI.EndChangeCheck())
                {
                    Undo.RecordObject(obstacle, "Edit obstacle");
                    var rawDelta = math.mul(math.inverse(obstacle.transform.rotation), newPos - pos);
                    Vector2 delta = (rawDelta / obstacle.transform.lossyScale).xz;

                    var l = obstacle.Vertices.Length - 1;
                    if (obstacle.Close && (_vertexIndex == 0 || _vertexIndex == l))
                    {
                        obstacle.Vertices[0] += delta;
                        obstacle.Vertices[l] += delta;
                    }
                    else
                    {
                        obstacle.Vertices[_vertexIndex] += delta;
                    }
                }
            }

            if (!obstacle.gameObject.activeInHierarchy || obstacle.Vertices == null || obstacle.Vertices.Length < 2)
                return;

            var t = Handles.color;
            Handles.color = DotsNavPrefs.EditColor;
            for (int i = 0; i < obstacle.Vertices.Length - 1; i++)
                Handles.DrawLine(obstacle.GetVertexWorldSpace(i), obstacle.GetVertexWorldSpace(i + 1));
            Handles.color = t;

            void CreateUp()
            {
                Undo.RecordObject(obstacle, "Edit obstacle");
                var l = obstacle.Vertices.ToList();

                if (obstacle.Vertices.Length == 2 && _vertexIndex == 1)
                {
                    l.Add(l[1]);
                    ++_vertexIndex;
                }
                else
                {
                    l.Insert(_vertexIndex, l[_vertexIndex]);
                    ++_vertexIndex;
                }

                obstacle.Vertices = l.ToArray();
            }

            void CreateDown()
            {
                Undo.RecordObject(obstacle, "Edit obstacle");
                var l = obstacle.Vertices.ToList();

                if (obstacle.Vertices.Length == 2 && _vertexIndex == 0)
                    l.Insert(0, l[0]);
                else
                    l.Insert(_vertexIndex, l[_vertexIndex]);

                obstacle.Vertices = l.ToArray();
            }

            void Remove()
            {
                if (obstacle.Vertices.Length > 0)
                {
                    Undo.RecordObject(obstacle, "Edit obstacle");
                    var l = obstacle.Vertices.ToList();
                    l.RemoveAt(_vertexIndex);
                    obstacle.Vertices = l.ToArray();
                    if (_vertexIndex == obstacle.Vertices.Length)
                        _vertexIndex = 0;
                }
            }
        }

        void DrawPoint(float2 pos)
        {
            const int size = 8;
            const float diag = size / 2.8284f;
            const int hor = size / 2;
            DrawLine(pos + new float2(-diag, -diag), pos + new float2(diag, diag));
            DrawLine(pos + new float2(-diag, diag), pos + new float2(diag, -diag));
            DrawLine(pos + new float2(-hor, 0), pos + new float2(hor, 0));
            DrawLine(pos + new float2(0, -hor), pos + new float2(0, hor));
        }

        static void DrawLine(float2 from, float2 to)
        {
            GL.Vertex(new float3(from, 1));
            GL.Vertex(new float3(to, 1));
        }
    }
}
#endif