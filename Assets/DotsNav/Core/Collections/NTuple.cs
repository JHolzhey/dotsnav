using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

// Value Tuples that are blittable and so can be used in Unity's Jobs
// Replace with HashCode with System.HashCode when available

namespace Unity.Collections
{

/* public static class NTuple
{
    public static NTuple<A, B> New<A, B>(A item1, B item2)
        where A : unmanaged
        where B : unmanaged
    {
        return new NTuple<A, B>(item1, item2);
    }

    public static NTuple<A, B, C> New<A, B, C>(A item1, B item2, C item3)
        where A : unmanaged
        where B : unmanaged
        where C : unmanaged
    {
        return new NTuple<A, B, C>(item1, item2, item3);
    }

    public static NTuple<A, B, C, D> New<A, B, C, D>(A item1, B item2, C item3, D item4)
        where A : unmanaged
        where B : unmanaged
        where C : unmanaged
        where D : unmanaged
    {
        return new NTuple<A, B, C, D>(item1, item2, item3, item4);
    }

    public static NTuple<A, B, C, D, E> New<A, B, C, D, E>(A item1, B item2, C item3, D item4, E item5)
        where A : unmanaged
        where B : unmanaged
        where C : unmanaged
        where D : unmanaged
        where E : unmanaged
    {
        return new NTuple<A, B, C, D, E>(item1, item2, item3, item4, item5);
    }
} */


[StructLayout(LayoutKind.Sequential)]
public struct NTuple<A, B> : IEquatable<NTuple<A, B>>, IEquatable<ValueTuple<A, B>>
    where A : unmanaged
    where B : unmanaged
{
    public A item1;
    public B item2;

    public NTuple(A item1, B item2)
    {
        this.item1 = item1;
        this.item2 = item2;
    }

    public bool Equals(NTuple<A, B> other) =>
        item1.Equals(other.item1) &&
        item2.Equals(other.item2);

    public bool Equals(ValueTuple<A, B> other) =>
        item1.Equals(other.Item1) &&
        item2.Equals(other.Item2);

    public override bool Equals(object obj) => (obj is NTuple<A, B> tuple) && Equals(tuple);

    public static bool operator ==(NTuple<A, B> left, NTuple<A, B> right) => left.Equals(right);

    public static bool operator !=(NTuple<A, B> left, NTuple<A, B> right) => !left.Equals(right);

    public static bool operator ==(NTuple<A, B> left, ValueTuple<A, B> right) => left.Equals(right);

    public static bool operator !=(NTuple<A, B> left, ValueTuple<A, B> right) => !left.Equals(right);

    public static bool operator ==(ValueTuple<A, B> left, NTuple<A, B> right) => left.Equals(right);

    public static bool operator !=(ValueTuple<A, B> left, NTuple<A, B> right) => !left.Equals(right);

    public override int GetHashCode() => HashCode.Combine(item1, item2);

    public static implicit operator ValueTuple<A, B>(NTuple<A, B> v) => new ValueTuple<A, B>(v.item1, v.item2);

    public static implicit operator NTuple<A, B>(ValueTuple<A, B> v) => new NTuple<A, B>(v.Item1, v.Item2);

    public void Unpack(out A item1, out B item2)
    {
        item1 = this.item1;
        item2 = this.item2;
    }
}

[StructLayout(LayoutKind.Sequential)]
public struct NTuple<A, B, C> : IEquatable<NTuple<A, B, C>>, IEquatable<ValueTuple<A, B, C>>
    where A : unmanaged
    where B : unmanaged
    where C : unmanaged
{
    public A item1;
    public B item2;
    public C item3;

    public NTuple(A item1, B item2, C item3)
    {
        this.item1 = item1;
        this.item2 = item2;
        this.item3 = item3;
    }

    public bool Equals(NTuple<A, B, C> other) =>
        item1.Equals(other.item1) &&
        item2.Equals(other.item2) &&
        item3.Equals(other.item3);

    public bool Equals(ValueTuple<A, B, C> other) =>
        item1.Equals(other.Item1) &&
        item2.Equals(other.Item2) &&
        item3.Equals(other.Item3);

    public override bool Equals(object obj) => (obj is NTuple<A, B, C> tuple) && Equals(tuple);

    public static bool operator ==(NTuple<A, B, C> left, NTuple<A, B, C> right) => left.Equals(right);

    public static bool operator !=(NTuple<A, B, C> left, NTuple<A, B, C> right) => !left.Equals(right);

    public static bool operator ==(NTuple<A, B, C> left, ValueTuple<A, B, C> right) => left.Equals(right);

    public static bool operator !=(NTuple<A, B, C> left, ValueTuple<A, B, C> right) => !left.Equals(right);

    public static bool operator ==(ValueTuple<A, B, C> left, NTuple<A, B, C> right) => left.Equals(right);

    public static bool operator !=(ValueTuple<A, B, C> left, NTuple<A, B, C> right) => !left.Equals(right);

    public override int GetHashCode() => HashCode.Combine(item1, item2, item3);

    public static implicit operator ValueTuple<A, B, C>(NTuple<A, B, C> v) =>
        new ValueTuple<A, B, C>(v.item1, v.item2, v.item3);

    public static implicit operator NTuple<A, B, C>(ValueTuple<A, B, C> v) =>
        new NTuple<A, B, C>(v.Item1, v.Item2, v.Item3);

    public void Unpack(out A item1, out B item2, out C item3)
    {
        item1 = this.item1;
        item2 = this.item2;
        item3 = this.item3;
    }
}

[StructLayout(LayoutKind.Sequential)]
public struct NTuple<A, B, C, D> : IEquatable<NTuple<A, B, C, D>>, IEquatable<ValueTuple<A, B, C, D>>
    where A : unmanaged
    where B : unmanaged
    where C : unmanaged
    where D : unmanaged
{
    public A item1;
    public B item2;
    public C item3;
    public D item4;

    public NTuple(A item1, B item2, C item3, D item4)
    {
        this.item1 = item1;
        this.item2 = item2;
        this.item3 = item3;
        this.item4 = item4;
    }

    public bool Equals(NTuple<A, B, C, D> other) =>
        item1.Equals(other.item1) &&
        item2.Equals(other.item2) &&
        item3.Equals(other.item3) &&
        item4.Equals(other.item4);

    public bool Equals(ValueTuple<A, B, C, D> other) =>
        item1.Equals(other.Item1) &&
        item2.Equals(other.Item2) &&
        item3.Equals(other.Item3) &&
        item4.Equals(other.Item4);

    public override bool Equals(object obj) =>
        (obj is NTuple<A, B, C, D> tuple) && Equals(tuple);

    public static bool operator ==(NTuple<A, B, C, D> left, NTuple<A, B, C, D> right) => left.Equals(right);

    public static bool operator !=(NTuple<A, B, C, D> left, NTuple<A, B, C, D> right) => !left.Equals(right);

    public static bool operator ==(NTuple<A, B, C, D> left, ValueTuple<A, B, C, D> right) => left.Equals(right);

    public static bool operator !=(NTuple<A, B, C, D> left, ValueTuple<A, B, C, D> right) => !left.Equals(right);

    public static bool operator ==(ValueTuple<A, B, C, D> left, NTuple<A, B, C, D> right) => left.Equals(right);

    public static bool operator !=(ValueTuple<A, B, C, D> left, NTuple<A, B, C, D> right) => !left.Equals(right);

    public override int GetHashCode() => HashCode.Combine(item1, item2, item3, item4);

    public static implicit operator ValueTuple<A, B, C, D>(NTuple<A, B, C, D> v) =>
        new ValueTuple<A, B, C, D>(v.item1, v.item2, v.item3, v.item4);

    public static implicit operator NTuple<A, B, C, D>(ValueTuple<A, B, C, D> v) =>
        new NTuple<A, B, C, D>(v.Item1, v.Item2, v.Item3, v.Item4);

    public void Unpack(out A item1, out B item2, out C item3, out D item4)
    {
        item1 = this.item1;
        item2 = this.item2;
        item3 = this.item3;
        item4 = this.item4;
    }
}

[StructLayout(LayoutKind.Sequential)]
public struct NTuple<A, B, C, D, E> : IEquatable<NTuple<A, B, C, D, E>>, IEquatable<ValueTuple<A, B, C, D, E>>
    where A : unmanaged
    where B : unmanaged
    where C : unmanaged
    where D : unmanaged
    where E : unmanaged
{
    public A item1;
    public B item2;
    public C item3;
    public D item4;
    public E item5;

    public NTuple(A item1, B item2, C item3, D item4, E item5)
    {
        this.item1 = item1;
        this.item2 = item2;
        this.item3 = item3;
        this.item4 = item4;
        this.item5 = item5;
    }

    public override bool Equals(object obj) => (obj is NTuple<A, B, C, D, E> tuple) && Equals(tuple);

    public bool Equals(NTuple<A, B, C, D, E> other) =>
        item1.Equals(other.item1) &&
        item2.Equals(other.item2) &&
        item3.Equals(other.item3) &&
        item4.Equals(other.item4) &&
        item5.Equals(other.item5);

    public bool Equals(ValueTuple<A, B, C, D, E> other) =>
        item1.Equals(other.Item1) &&
        item2.Equals(other.Item2) &&
        item3.Equals(other.Item3) &&
        item4.Equals(other.Item4) &&
        item5.Equals(other.Item5);

    public static bool operator ==(NTuple<A, B, C, D, E> left, NTuple<A, B, C, D, E> right) => left.Equals(right);

    public static bool operator !=(NTuple<A, B, C, D, E> left, NTuple<A, B, C, D, E> right) => !left.Equals(right);

    public static bool operator ==(NTuple<A, B, C, D, E> left, ValueTuple<A, B, C, D, E> right) => left.Equals(right);

    public static bool operator !=(NTuple<A, B, C, D, E> left, ValueTuple<A, B, C, D, E> right) => !left.Equals(right);

    public static bool operator ==(ValueTuple<A, B, C, D, E> left, NTuple<A, B, C, D, E> right) => left.Equals(right);

    public static bool operator !=(ValueTuple<A, B, C, D, E> left, NTuple<A, B, C, D, E> right) => !left.Equals(right);

    public override int GetHashCode() => HashCode.Combine(item1, item2, item3, item4, item5);

    public static implicit operator ValueTuple<A, B, C, D, E>(NTuple<A, B, C, D, E> v) =>
        new ValueTuple<A, B, C, D, E>(v.item1, v.item2, v.item3, v.item4, v.item5);

    public static implicit operator NTuple<A, B, C, D, E>(ValueTuple<A, B, C, D, E> v) =>
        new NTuple<A, B, C, D, E>(v.Item1, v.Item2, v.Item3, v.Item4, v.Item5);

    public void Unpack(out A item1, out B item2, out C item3, out D item4, out E item5)
    {
        item1 = this.item1;
        item2 = this.item2;
        item3 = this.item3;
        item4 = this.item4;
        item5 = this.item5;
    }
}

public static class HashCode
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Combine<A,B>(A item1, B item2)
    {
        return CombineHashCodes(item1.GetHashCode(), item2.GetHashCode());
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Combine<A,B,C>(A item1, B item2, C item3)
    {
        return CombineHashCodes(item1.GetHashCode(), item2.GetHashCode(), item3.GetHashCode());
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Combine<A,B,C,D>(A item1, B item2, C item3, D item4)
    {
        return CombineHashCodes(item1.GetHashCode(), item2.GetHashCode(), item3.GetHashCode(), item4.GetHashCode());
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Combine<A,B,C,D,E>(A item1, B item2, C item3, D item4, E item5)
    {
        return CombineHashCodes(item1.GetHashCode(), item2.GetHashCode(), item3.GetHashCode(), item4.GetHashCode(), item5.GetHashCode());
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Combine<A,B,C,D,E,F>(A item1, B item2, C item3, D item4, E item5, F item6)
    {
        return CombineHashCodes(item1.GetHashCode(), item2.GetHashCode(), item3.GetHashCode(), item4.GetHashCode(), item5.GetHashCode(),
                                item6.GetHashCode());
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Combine<A,B,C,D,E,F,G>(A item1, B item2, C item3, D item4, E item5, F item6, G item7)
    {
        return CombineHashCodes(item1.GetHashCode(), item2.GetHashCode(), item3.GetHashCode(), item4.GetHashCode(), item5.GetHashCode(),
                                item6.GetHashCode(), item7.GetHashCode());
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Combine<A,B,C,D,E,F,G,H>(A item1, B item2, C item3, D item4, E item5, F item6, G item7, H item8)
    {
        return CombineHashCodes(item1.GetHashCode(), item2.GetHashCode(), item3.GetHashCode(), item4.GetHashCode(), item5.GetHashCode(),
                                item6.GetHashCode(), item7.GetHashCode(), item8.GetHashCode());
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Combine<A,B,C,D,E,F,G,H,I>(A item1, B item2, C item3, D item4, E item5, F item6, G item7, H item8, I item9)
    {
        return CombineHashCodes(item1.GetHashCode(), item2.GetHashCode(), item3.GetHashCode(), item4.GetHashCode(), item5.GetHashCode(),
                                item6.GetHashCode(), item7.GetHashCode(), item8.GetHashCode(), item9.GetHashCode());
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Combine<A,B,C,D,E,F,G,H,I,J>(A item1, B item2, C item3, D item4, E item5, F item6, G item7, H item8, I item9, J item10)
    {
        return CombineHashCodes(item1.GetHashCode(), item2.GetHashCode(), item3.GetHashCode(), item4.GetHashCode(), item5.GetHashCode(),
                                item6.GetHashCode(), item7.GetHashCode(), item8.GetHashCode(), item9.GetHashCode(), item10.GetHashCode());
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Combine<A,B,C,D,E,F,G,H,I,J,K>(A item1, B item2, C item3, D item4, E item5, F item6, G item7, H item8, I item9, J item10, K item11)
    {
        return CombineHashCodes(item1.GetHashCode(), item2.GetHashCode(), item3.GetHashCode(), item4.GetHashCode(), item5.GetHashCode(),
            item6.GetHashCode(), item7.GetHashCode(), item8.GetHashCode(), item9.GetHashCode(), item10.GetHashCode(), item11.GetHashCode());
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CombineHashCodes(int h1, int h2)
    {
        // https://stackoverflow.com/questions/1646807/quick-and-simple-hash-code-combinations
        var hash = 17;
        hash = hash * 31 + h1;
        hash = hash * 31 + h2;
        return hash;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CombineHashCodes(int h1, int h2, int h3)
    {
        return CombineHashCodes(CombineHashCodes(h1, h2), h3);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CombineHashCodes(int h1, int h2, int h3, int h4)
    {
        return CombineHashCodes(CombineHashCodes(h1, h2), CombineHashCodes(h3, h4));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CombineHashCodes(int h1, int h2, int h3, int h4, int h5)
    {
        return CombineHashCodes(CombineHashCodes(h1, h2, h3, h4), h5);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CombineHashCodes(int h1, int h2, int h3, int h4, int h5, int h6)
    {
        return CombineHashCodes(CombineHashCodes(h1, h2, h3, h4), CombineHashCodes(h5, h6));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CombineHashCodes(int h1, int h2, int h3, int h4, int h5, int h6, int h7)
    {
        return CombineHashCodes(CombineHashCodes(h1, h2, h3, h4), CombineHashCodes(h5, h6, h7));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CombineHashCodes(int h1, int h2, int h3, int h4, int h5, int h6, int h7, int h8)
    {
        return CombineHashCodes(CombineHashCodes(h1, h2, h3, h4), CombineHashCodes(h5, h6, h7, h8));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CombineHashCodes(int h1, int h2, int h3, int h4, int h5, int h6, int h7, int h8, int h9)
    {
        return CombineHashCodes(CombineHashCodes(h1, h2, h3, h4), CombineHashCodes(h5, h6, h7, h8), h9);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CombineHashCodes(int h1, int h2, int h3, int h4, int h5, int h6, int h7, int h8, int h9, int h10)
    {
        return CombineHashCodes(CombineHashCodes(h1, h2, h3, h4), CombineHashCodes(h5, h6, h7, h8), h9, h10);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CombineHashCodes(int h1, int h2, int h3, int h4, int h5, int h6, int h7, int h8, int h9, int h10, int h11)
    {
        return CombineHashCodes(CombineHashCodes(h1, h2, h3, h4), CombineHashCodes(h5, h6, h7, h8), h9, h10, h11);
    }
}
}