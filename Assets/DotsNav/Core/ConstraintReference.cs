using System;
using Unity.Entities;

namespace DotsNav
{
    public enum ConstraintType : byte
    {
        None = 0,
        Obstacle,
        Terrain,
        Gate,
        Link,
    }

    /// <summary>
    /// Used to indicate which obstacle should be removed. Obtained when inserting obstacles.
    /// </summary>
    public struct ConstraintReference : IEquatable<ConstraintReference>, IComparable<ConstraintReference>
    {
        /// <summary>
        /// The bounding box and bulk inserted obstacles are static and cannot be removed from the navmesh
        /// </summary>
        public bool IsStatic => Value == Entity.Null;

        internal Entity Value;

        internal ConstraintReference(Entity entity)
        {
            Value = entity;
        }

        public bool Equals(ConstraintReference other) => Value.Equals(other.Value);
        public int CompareTo(ConstraintReference other) => Value.CompareTo(other.Value);
    }
}