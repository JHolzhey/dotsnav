using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace DotsNav.Navmesh.Data
{
    public struct NavmeshMaterialType {
        public FixedString32Bytes name;
        public readonly float cost;
        public readonly float slopeCostFactor;
        public readonly UnityEngine.Color color;
        public NavmeshMaterialType(FixedString32Bytes name, float cost, UnityEngine.Color color, float slopeCostFactor = 1f) {
            this.name = name;
            this.cost = cost;
            this.slopeCostFactor = slopeCostFactor;
            this.color = color;
        }
    }
    /// <summary>
    /// Create to trigger creation of a navmesh. Destroy to trigger destruction of a navmesh.
    /// </summary>
    public unsafe struct NavmeshComponent : IComponentData, IEquatable<NavmeshComponent>
    {
        /// <summary>
        /// Size of the navmesh to be created. The navmesh will be centered around the origin
        /// </summary>
        public readonly float2 Size;

        /// <summary>
        /// Vertices inserted with this range of an existing vertex are merged instead
        /// </summary>
        public readonly float MergePointsDistance;

        /// <summary>
        /// Margin for considering points collinear when removing intersections
        /// </summary>
        public readonly float CollinearMargin;

        /// <summary>
        /// Used to determine the sizes of initial allocations
        /// </summary>
        public readonly int ExpectedVerts;

        public readonly UnsafeList<NavmeshMaterialType> MaterialTypes;

        public NavmeshComponent(float2 size, int expectedVerts, UnsafeList<NavmeshMaterialType> materialTypes, float mergePointsDistance = 1e-3f, float collinearMargin = 1e-6f)
        {
            UnityEngine.Debug.Assert(materialTypes.Length < 30, "Currently cannot have more than X Material Types");
            Size = size;
            ExpectedVerts = expectedVerts;
            MaterialTypes = materialTypes;
            MergePointsDistance = mergePointsDistance;
            CollinearMargin = collinearMargin;
            Navmesh = default;
        }

        // todo make non public
        public Navmesh* Navmesh;

        public bool Equals(NavmeshComponent other) => Navmesh == other.Navmesh;
    }
}