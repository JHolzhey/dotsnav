using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;


namespace Unity.Mathematics
{
public struct Triangle : IGeometry {
    public float3 p0;
    public float3 p1;
    public float3 p2;

    public Plane Plane => new Plane(Normal, p0);
    
    public Triangle(float3 p0, float3 p1, float3 p2) {
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
    }

    public readonly float3 Center => Centroid();
    public readonly float4x4 CalcLocalMatrix() => MathLib.CalcLocalMatrix(Normal, Center);
    public readonly float3 Centroid() => MathLib.TriangleCentroid(p0, p1, p2);
    public readonly float3 Normal => MathLib.CalcTriangleNormalCCW(p0, p1, p2);

    public Triangle RoughInset(float insetAmount) {
        MathLib.InsetTriangle(p0, p1, p2, insetAmount, out float3 newP0, out float3 newP1, out float3 newP2);
        return new Triangle(newP0, newP1, newP2);
    }

    public bool IsValid() => p0.IsPhysicallyValid() && p1.IsPhysicallyValid() && p2.IsPhysicallyValid();
}
}