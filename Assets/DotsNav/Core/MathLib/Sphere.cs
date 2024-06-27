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
public struct Sphere : IComponentData, IShape
{
    public float3 center;
    public float radius;
    public float Diameter => radius*2f;

    public Sphere(float3 center, float radius) {
        this.center = center;
        this.radius = radius;
    }

    public void Update(float3 center) => this.center = center;
    public void UpdateRadius(float radius) => this.radius = radius;

    public float3 Center => center;
    public float4x4 CalcLocalMatrix() => MathLib.CalcLocalMatrix(math.up(), center);

    public void ToAABB(out float3 minPosition, out float3 maxPosition) => ToAABB(out minPosition, out maxPosition);
    public void ToAABB(out float3 minPosition, out float3 maxPosition, float radiusAdd = 0f) {
        MathLib.SphereToAABB(center, radius + radiusAdd, out minPosition, out maxPosition);
    }
    public Rect ToBoundsRect() {
        ToAABB(out float3 minPosition, out float3 maxPosition);
        return Rect.MinMaxRect(minPosition.x, minPosition.z, maxPosition.x, maxPosition.y);
    }

    public void Enlarge(float amount) => UpdateRadius(radius + amount);

    public bool IsValid() => center.IsPhysicallyValid() && radius.IsPhysicallyValid();
}
}