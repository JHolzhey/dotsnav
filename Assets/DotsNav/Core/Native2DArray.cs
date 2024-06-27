using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;


namespace Unity.Collections
{
public struct Native2DArray<T> : IDisposable, IEnumerable<T>, IEnumerable
    where T : unmanaged
{
    public NativeArray<T> flatArray;
    public int LengthX { get; private set; }
    public int LengthY => flatArray.Length / LengthX;
    public int Length => flatArray.Length;
 
    public Native2DArray(int sizeX, int sizeY, Allocator allocator, NativeArrayOptions options = NativeArrayOptions.ClearMemory) {
        Debug.Assert(sizeX > 0 && sizeY > 0);
        this.flatArray = new NativeArray<T>(sizeX * sizeY, allocator, options);
        this.LengthX = sizeX;
    }

    public int Index2DTo1D(int x, int y) => MathLib.Index2DTo1D(x, y, LengthX);
    public int Index2DTo1D(int2 xy) => Index2DTo1D(xy.x, xy.y);
    

    public T this[int flatIndex] {
        get { return flatArray[flatIndex]; }
        set { flatArray[flatIndex] = value; }
    }

    public T this[int x, int y] {
        get { return flatArray[Index2DTo1D(x, y)]; }
        set { flatArray[Index2DTo1D(x, y)] = value; }
    }
    public T this[int2 xy] {
        get { return this[xy.x, xy.y]; }
        set { this[xy.x, xy.y] = value; }
    }

    public NativeArray<T>.Enumerator GetEnumerator() => flatArray.GetEnumerator();
    IEnumerator<T> IEnumerable<T>.GetEnumerator() => GetEnumerator();
    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
 
    public void Dispose() {
        flatArray.Dispose();
    }
}
}