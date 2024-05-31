﻿using System.Diagnostics;
using System.Runtime.InteropServices;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

namespace DotsNav.Navmesh
{
    /// <summary>
    /// Represents an undirected edge in the navmesh. For more information see:
    /// <para>Primitives for the Manipulation of General Subdivisions and the Computation of Voronoi Diagrams, Guibas and Stolfi (1985).</para>
    /// <para>http://sccg.sk/~samuelcik/dgs/quad_edge.pdf</para>
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    struct QuadEdge
    {
        public Edge Edge0;
        public Edge Edge1;
        public Edge Edge2;
        public Edge Edge3;
        public UnsafeList<Entity> Crep;
        public int Mark;
        public int Id;
        public bool RefineFailed;

        unsafe Edge* _majorEdge;
        public unsafe Edge* MajorEdge {
            get {
                // UnityEngine.Debug.Assert(MathLib.IfFirstCheckSecond(_majorEdge != null, EdgeType == Edge.Type.ConnectsToTerrainWithMajorConnectsToObstacle));
                return _majorEdge;
            }
            set {
                // UnityEngine.Debug.Assert(value->EdgeType == Edge.Type.ConnectsToObstacleWithMinorTerrainConnects);
                _majorEdge = value;
            }
        }

        public Edge.Type EdgeType;
    }
}