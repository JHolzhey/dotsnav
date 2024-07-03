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
public struct PlanePoint : IGeometry {
    public float2 point;
    public float height;
    public readonly float3 Point3D => new float3(point.x, height, point.y);
    public readonly bool Is3D => height != float.NegativeInfinity;
    public readonly float SafeHeight => Is3D ? height : 0f;
    
    public PlanePoint(float3 point3D) {
        this.point = point3D.xz;
        this.height = point3D.y;
    }
    public PlanePoint(float2 point) {
        this.point = point;
        this.height = float.NegativeInfinity;
    }
    public PlanePoint(float2 point, float height) {
        this.point = point;
        this.height = height;
    }

    public readonly float3 Center => Point3D;
    public readonly float4x4 CalcLocalMatrix() => MathLib.CalcLocalMatrix(math.up(), Center);
    public readonly float3 Centroid() => Center;


    public static implicit operator float3(PlanePoint e) => e.Point3D;
    public static implicit operator PlanePoint(float3 v) => new (v);

    public static implicit operator float2(PlanePoint e) => e.point;
    public static implicit operator PlanePoint(float2 v) => new (v);

    public bool IsValid() => point.IsPhysicallyValid() && height.IsPhysicallyValid();
}
}