using Unity.Entities;
using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;

namespace DotsNav.Data
{
    public readonly struct NavmeshMaterialCost {
        public readonly float cost;
        public readonly float slopeCostFractionIgnored; // 1 == completely ignored, 0 (default) == completely used
        public NavmeshMaterialCost(float cost, float slopeCostFractionIgnored = default) {
            this.cost = cost;
            this.slopeCostFractionIgnored = slopeCostFractionIgnored;
        }
    }

    public struct AgentComponent : IComponentData
    {
        public FloatRange Radius;
        public float Depth;
        public UnsafeList<NavmeshMaterialCost> MaterialCosts; // Will override NavMesh.DefaultMaterialCosts

        public AgentComponent(FloatRange radius, float depth, UnsafeList<NavmeshMaterialCost> materialCosts)
        {
            Radius = radius;
            Depth = depth;
            MaterialCosts = materialCosts;
        }

        public AgentComponent(FloatRange radius, float depth = 1f) : this(radius, depth, default) {}
        public AgentComponent(float radius, float depth = 1f) : this(new FloatRange(radius, radius), depth, default) {}
        
        public static implicit operator float(AgentComponent e) => e.Radius.min;
        public static implicit operator AgentComponent(float v) => new(v);
    }
}