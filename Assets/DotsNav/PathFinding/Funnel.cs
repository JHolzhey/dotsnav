using System;
using DotsNav.Collections;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using DotsNav.Drawing;
using Unity.Collections.LowLevel.Unsafe;

namespace DotsNav.PathFinding
{
    struct Funnel
    {
        readonly Deque<Node> _funnel;
        public bool IsCreated => _funnel.IsCreated;
        Node _apex;
        Deque<Node> _output;

        UnsafeHashMap<float2, float> _debugCircleCenters;
        // float _radius;

        public Funnel(int capacity, Allocator allocator) : this()
        {
            _funnel = new Deque<Node>(capacity, allocator);

            _debugCircleCenters = new UnsafeHashMap<float2, float>(1, allocator);
        }

        public void GetPath(List<Gate> channel, float2 start, float2 goal, float maxRadius, Deque<Node> output)
        {
            _debugCircleCenters.Clear();
            _funnel.Clear();
            // _radius = radius.min;
            _output = output;

            if (channel.Length == 0)
            {
                _output.PushBack(new Node(goal, start, goal, maxRadius, NodeType.Point));
                return;
            }
            var first = channel[0];
            _apex = new Node(start, start, start, 0, NodeType.Point);
            AddLeft(_apex);
            AddL(first.Left, first.Radius);
            AddR(first.Right, first.Radius);

            for (int i = 1; i < channel.Length; i++)
            {
                Assert.IsTrue(math.all(channel[i].Left == channel[i - 1].Left) != math.all(channel[i].Right == channel[i - 1].Right));

                var toInsert = channel[i];

                var addRight = math.all(toInsert.Left == channel[i - 1].Left);
                var p = addRight ? toInsert.Right : toInsert.Left;

                if (toInsert.IsGoalGate && math.distancesq(_apex.Vertex, goal) < math.distancesq(_apex.Vertex, p))
                    break;
                
                float radius = toInsert.Radius;
                int i_Next = i + 1;
                if (addRight) {
                    while (i_Next != channel.Length && math.all(channel[i_Next].Right == toInsert.Right)) {
                        radius = math.min(radius, channel[i_Next].Radius);
                        i_Next++;
                    }
                    Circle.Draw(p.ToXxY(), radius, Color.white);
                    AddR(p, radius);
                }
                else {
                    while (i_Next != channel.Length && math.all(channel[i_Next].Left == toInsert.Left)) {
                        radius = math.min(radius, channel[i_Next].Radius);
                        i_Next++;
                    }
                    Circle.Draw(p.ToXxY(), radius, Color.red);
                    AddL(p, radius);
                }
            }

            AddP(goal); // channel[^1].Radius); TODO: I believe these are equivalent
        }

        void AddL(float2 v, float radius)
        {
            while (true)
            {
                PathSegment(ref _debugCircleCenters, Left.Vertex, v, Left.Type, NodeType.Left, Left.Radius, radius, out var from, out var to);

                var add = math.all(Left.Vertex == Right.Vertex);

                if (!add)
                {
                    var node = math.all(Left.Vertex == _apex.Vertex) ? FromLeft(1) : Left;
                    add = Math.Angle(node.To - node.From, to - from) <= 0;
                }

                if (add)
                {
                    AddLeft(new Node(v, from, to, radius, NodeType.Left));
                    return;
                }
                // else:
                if (math.all(Left.Vertex == _apex.Vertex))
                {
                    AddToPath(_apex);

                    var f = FromLeft(1).From;
                    var t = FromLeft(1).To;
                    if (math.distancesq(from, to) < math.distancesq(f, t) && DistanceToLineSq(f, t, to) <= Math.Square(radius))
                    {
                        PopLeft();

                        var toReplace = PopLeft();
                        PathSegment(ref _debugCircleCenters, v, toReplace.Vertex, NodeType.Left, NodeType.Right, radius, toReplace.Radius, out var from1, out var to1);
                        AddLeft(new Node(toReplace.Vertex, from1, to1, radius, NodeType.Right));

                        AddLeft(new Node(v, from, to, radius, NodeType.Left));
                        _apex = Left;
                        return;
                    }

                    _apex = FromLeft(1);
                }

                PopLeft();
            }
        }

        void AddR(float2 v, float radius, NodeType type = NodeType.Right)
        {
            while (true)
            {
                PathSegment(ref _debugCircleCenters, Right.Vertex, v, Right.Type, type, Right.Radius, radius, out var from, out var to);

                var add = math.all(Right.Vertex == Left.Vertex);

                if (!add)
                {
                    var node = math.all(Right.Vertex == _apex.Vertex) ? FromRight(1) : Right;
                    add = Math.Angle(node.To - node.From, to - from) >= 0;
                }

                if (add)
                {
                    AddRight(new Node(v, from, to, radius, type));
                    return;
                }
                // else:
                if (math.all(Right.Vertex == _apex.Vertex))
                {
                    AddToPath(_apex);

                    var f = FromRight(1).From;
                    var t = FromRight(1).To;
                    if (math.distancesq(from, to) < math.distancesq(f, t) && DistanceToLineSq(f, t, to) <= Math.Square(radius))
                    {
                        PopRight();

                        var toReplace = PopRight();
                        PathSegment(ref _debugCircleCenters, v, toReplace.Vertex, NodeType.Right, NodeType.Left, radius, toReplace.Radius, out var from1, out var to1);
                        AddRight(new Node(toReplace.Vertex, from1, to1, radius, NodeType.Left));

                        AddRight(new Node(v, from, to, radius, type));
                        _apex = Right;
                        return;
                    }

                    _apex = FromRight(1);
                }

                PopRight();
            }
        }

        static float DistanceToLineSq(double2 a, double2 b, double2 p)
        {
            var ap = p - a;
            var ab = b - a;
            var f = math.dot(ap, ab) / math.lengthsq(ab);
            return (float) math.distancesq(p, a + ab * f);
        }

        void AddP(float2 v)
        {
            AddR(v, 0, NodeType.Point);

            for (var i = 0; i < _funnel.Count; i++)
            {
                if (math.all(FromLeft(i).Vertex == _apex.Vertex))
                {
                    for (; i < _funnel.Count; i++)
                        AddToPath(FromLeft(i));
                    break;
                }
            }

            _output.PopFront();
        }

        void AddToPath(Node node)
        {
            while (_output.Count > 1 && _output.Back.Type == NodeType.Left != Math.Ccw(_output.Back.From, _output.Back.To, node.To))
            {
                _output.PopBack();
                PathSegment(ref _debugCircleCenters, _output.Back.Vertex, node.Vertex, _output.Back.Type, node.Type, _output.Back.Radius, node.Radius /* TODO: should this be node.Radius???? */, out var from, out var to);
                node.To = to;
                node.From = from;
            }

            _output.PushBack(node);
        }

        static void PathSegment(ref UnsafeHashMap<float2, float> debugCircleCenters, float2 v0, float2 v1, NodeType t0, NodeType t1, float r0, float r1, out float2 from, out float2 to)
        {
            switch (t0)
            {
                case NodeType.Point:
                    Assert.IsTrue(r0 == 0, "A point should have a radius of 0");
                    from = v0;
                    switch (t1)
                    {
                        case NodeType.Point:
                            to = v1;
                            // DebugCheck(ref debugCircleCenters, v0, v1, r0, r1, from, to, Color.green);
                            return;
                        case NodeType.Left:
                            to = Math.GetTangentRight(v0, v1, r1);
                            // DebugCheck(ref debugCircleCenters, v0, v1, r0, r1, from, to, Color.green);
                            return;
                        case NodeType.Right:
                            to = Math.GetTangentLeft(v0, v1, r1);
                            // DebugCheck(ref debugCircleCenters, v0, v1, r0, r1, from, to, Color.green);
                            return;
                    }

                    break;
                case NodeType.Left:
                    switch (t1)
                    {
                        case NodeType.Point:
                            Assert.IsTrue(r1 == 0, "A point should have a radius of 0");
                            to = v1;
                            from = Math.GetTangentLeft(v1, v0, r0);
                            // DebugCheck(ref debugCircleCenters, v0, v1, r0, r1, from, to, Color.green);
                            return;
                        case NodeType.Left:
                            Math.GetOuterTangentRight(v0, r0, v1, r1, out from, out to);
                            DebugCheck(ref debugCircleCenters, v0, v1, r0, r1, from, to, Color.magenta);
                            return;
                        case NodeType.Right:
                            Math.GetInnerTangentRight(v0, r0, v1, r1, out from, out to);
                            DebugCheck(ref debugCircleCenters, v0, v1, r0, r1, from, to, Color.yellow);
                            return;
                    }

                    break;
                case NodeType.Right:
                    switch (t1)
                    {
                        case NodeType.Point:
                            Assert.IsTrue(r1 == 0, "A point should have a radius of 0");
                            to = v1;
                            from = Math.GetTangentRight(v1, v0, r0);
                            // DebugCheck(ref debugCircleCenters, v0, v1, r0, r1, from, to, Color.green);
                            return;
                        case NodeType.Left:
                            Math.GetInnerTangentLeft(v0, r0, v1, r1, out from, out to);
                            DebugCheck(ref debugCircleCenters, v0, v1, r0, r1, from, to, Color.white);
                            return;
                        case NodeType.Right:
                            Math.GetOuterTangentLeft(v0, r0, v1, r1, out from, out to);
                            DebugCheck(ref debugCircleCenters, v0, v1, r0, r1, from, to, Color.red);
                            return;
                    }

                    break;
            }

            throw new ArgumentOutOfRangeException();

            void DebugCheck(ref UnsafeHashMap<float2, float> debugCircleCenters, float2 v0, float2 v1, float r0, float r1, float2 from, float2 to, Color color) {
                if (debugCircleCenters.TryGetValue(v0, out float item) && item != r0) {
                    Circle.Draw(v0.ToXxY(), r0 == 0 ? 0.003f : r0 + 0.002f, Color.black);
                    Debug.Assert(false, "1 Circle visited with different radius");
                }
                if (debugCircleCenters.TryGetValue(v1, out item) && item != r1) {
                    Circle.Draw(v1.ToXxY(), r1 == 0 ? 0.003f : r1 + 0.002f, Color.black);
                    Debug.Assert(false, "2 Circle visited with different radius");
                }
                debugCircleCenters.TryAdd(v0, r0);
                debugCircleCenters.TryAdd(v1, r1);

                Circle.Draw(v0.ToXxY(), r0 == 0 ? 0.001f : r0, color);
                Circle.Draw(v1.ToXxY(), r1 == 0 ? 0.001f : r1, color);
                Line.Draw(from.ToXxY(), to.ToXxY(), color);
            }
        }

        void AddLeft(Node v) => _funnel.PushFront(v);
        void AddRight(Node v) => _funnel.PushBack(v);
        Node PopLeft() => _funnel.PopFront();
        Node PopRight() => _funnel.PopBack();
        Node FromLeft(int index) => _funnel.FromFront(index);
        Node FromRight(int index) => _funnel.FromBack(index);
        Node Left => _funnel.Front;
        Node Right => _funnel.Back;

        public void Clear()
        {
            _funnel.Clear();
        }
    
        public void Dispose()
        {
            _funnel.Dispose();
        }

        public struct Node
        {
            public readonly float2 Vertex;
            public float2 From;
            public float2 To;
            public float Radius;
            public readonly NodeType Type;

            public float Length => math.length(To - From);
        
            public Node(float2 vertex, float2 from, float2 to, float radius, NodeType type)
            {
                Vertex = vertex;
                From = from;
                To = to;
                Radius = radius;
                Type = type;
            }

            public override string ToString() => $"{Type} node at {Vertex}";
        }

        public enum NodeType
        {
            Point,
            Left,
            Right
        }
    }
}