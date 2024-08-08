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
public struct Seg : IComponentData, IGeometry
{
    public float3 start;
    public float3 end;
    
    public readonly bool IsVertical => start.x == end.x && start.z == end.z;
    public readonly float3 Vector => end - start;
    public readonly float3 Direction => math.normalize(Vector);
    public readonly float3 Forward => Direction;
    public readonly float Length => math.length(Vector);
    public readonly float3 Right => MathLib.CalcTangentToNormal(Direction);
    public readonly float3 Up => math.normalize(math.cross(RightXZ, Vector)); // TODO: Untested
    public readonly float3 Midpoint => MathLib.Midpoint(start, end);

    public readonly float3 DirectionXZ => IsVertical ? float3.zero : math.normalizesafe(Vector.XOZ());

    // TODO: Make sure its the same as 'Right' when horizontal and vertical
    // TODO: Replace MathLib.CalcTangentToNormal() with the below? Is it not the exact same and yet faster?
    public readonly float3 RightXZ => IsVertical ? math.right() : math.normalize(new float3(-Vector.z, 0, Vector.x));

    public readonly float SlopeAngle => MathLib.VectorSlopeAngle(Vector);

    public readonly Plane UpPlane => new Plane(Up, start); // TODO: Pretty sure this doesn't work

    public Seg(float3 start, float3 end) {
        this.start = start;
        this.end = end;
        Update(start, end);
    }

    public void Update(float3 start, float3 end) {
        this.start = start;
        this.end = end;
        
        if (start.Equals(end)) {
            this.end.x += 0.001f;
        }
    }

    public float3 Center => Midpoint;
    public float4x4 CalcLocalMatrix() => MathLib.CalcLocalMatrix(Direction, start);


    public float3 PointGivenXZ(float2 xz) { // TODO: Make Line struct and make this a method. Also should improve speed of this method
        float3 direction = Direction;
        float3 pointOnLine = start; // Value taken if IsVertical
        bool success = true;
        if (direction.x != 0) {
            success = MathLib.IsPointOnLineGivenX(start, Direction, xz.x, out _, out pointOnLine);
        } else if (direction.z != 0) {
            success = MathLib.IsPointOnLineGivenZ(start, Direction, xz.y, out _, out pointOnLine);
        } else {
            Debug.Assert(IsVertical);
        }
        Debug.Assert(success, "Should not be possible");
        return pointOnLine;
    }

    public void ExtendForward(float amount) => Update(start, end + Direction*amount);
    public void ExtendBackward(float amount) => Update(start - Direction*amount, end);
    public void Extend(float amount) => Update(start - Direction*amount*0.5f, end + Direction*amount*0.5f);
    public void Shorten(float amount) => Extend(-amount);
    public float3 GetPointRay(float distanceFromStart) => start + Direction * distanceFromStart;
    public float3 GetPointSeg(float distanceFromStart) => start + Direction * math.clamp(distanceFromStart, 0, Length);

    public bool IsValid() => start.IsPhysicallyValid() && end.IsPhysicallyValid();
}
}