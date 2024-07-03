using Unity.Entities;
using Unity.Mathematics;

namespace DotsNav.Data
{
    /// <summary>
    /// Add DynamicBuffer&lt;VertexElement&gt; in addition to ObstacleData to queue insertion of obstacles,
    /// or in addition to DynamicBuffer&lt;VertexAmountElement&gt; to queue bulk insertion of permanent obstacles
    /// </summary>
    [InternalBufferCapacity(0)]
    public struct VertexElement : IBufferElementData
    {
        public PlanePoint Value;

        VertexElement(float2 v) {
            Value = new PlanePoint(v);
        }
        VertexElement(float3 v) {
            Value = new PlanePoint(v);
        }

        public static implicit operator float3(VertexElement e) => e.Value;
        public static implicit operator VertexElement(float3 v) => new (v);

        public static implicit operator float2(VertexElement e) => e.Value.point;
        public static implicit operator VertexElement(float2 v) => new (v);
    }
}