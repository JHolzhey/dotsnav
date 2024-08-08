using System.Diagnostics;
using Unity.Mathematics;

namespace DotsNav.PathFinding
{
    [DebuggerDisplay("{DebuggerDisplay,nq}")]
    struct Gate
    {
        string DebuggerDisplay => $"Left: {Left.x:0.00}, {Left.y:0.00}  Right: {Right.x:0.00}, {Right.y:0.00}, Radius: {Radius}";

        public float2 Left;
        public float2 Right;
        public float Radius;
        public bool IsGoalGate;

        public Gate(float2 left, float2 right, float radius, bool isGoalGate = false) {
            Left = left;
            Right = right;
            Radius = radius;
            IsGoalGate = isGoalGate;
        }
    }
}