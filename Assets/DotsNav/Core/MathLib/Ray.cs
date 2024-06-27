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
// public struct Ray : IComponentData, ITransform
// {
//     public float3 start;
//     public float3 end;
//     public float radius;
//     // public float innerRadius;
    
//     public bool IsVertical => start.x == end.x && start.z == end.z;
//     public readonly float3 Vector => end - start;
//     public readonly float3 Direction => math.normalize(Vector);
//     public readonly float Length => math.length(Vector);
//     public readonly float3 Right => MathLib.CalcTangentToNormal(Direction);
//     public readonly float3 Midpoint => MathLib.Midpoint(start, end);
// }
}