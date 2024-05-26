using System;
using Unity.Entities;

namespace DotsNav.Navmesh.Data
{
    /// <summary>
    /// Create to trigger insertion of an obstacle. Destroy to trigger removal of an obstacle.
    /// Add a DynamicBuffer&lt;VertexElement&gt; or VertexBlobComponent to supply vertices
    /// </summary>
    public struct NavmeshObstacleComponent : IComponentData
    {
        public ConstraintType constraintType;
        public NavmeshObstacleComponent(ConstraintType constraintType) {
            this.constraintType = constraintType;
        }
    }
}