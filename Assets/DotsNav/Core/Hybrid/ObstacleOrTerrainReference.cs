using System;
using Unity.Entities;

namespace DotsNav.Core.Hybrid
{
    /// <summary>
    /// Used to indicate which obstacle should be removed. Obtained when inserting obstacles.
    /// </summary>
    public struct ObstacleOrTerrainReference : IEquatable<ObstacleOrTerrainReference>, IComparable<ObstacleOrTerrainReference>
    {
        /// <summary>
        /// The bounding box and bulk inserted obstacles are static and cannot be removed from the navmesh
        /// </summary>
        public bool IsStatic => Value == Entity.Null;

        internal Entity Value;

        internal ObstacleOrTerrainReference(Entity entity)
        {
            Value = entity;
        }

        public bool Equals(ObstacleOrTerrainReference other) => Value.Equals(other.Value);
        public int CompareTo(ObstacleOrTerrainReference other) => Value.CompareTo(other.Value);
    }
}