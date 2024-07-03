using System;

namespace Unity.Collections
{
public struct Pair<T, U> : IEquatable<Pair<T, U>>
    where T : unmanaged
    where U : unmanaged
{
    public Pair(T first, U second) {
        this.first = first;
        this.second = second;
    }
    public T first;
    public U second;

    public bool Equals(Pair<T, U> other) => first.Equals(other.first) && second.Equals(other.second);
    public override int GetHashCode() => HashCode.Combine(first, second);

    public static bool operator ==(Pair<T, U> left, Pair<T, U> right) => left.Equals(right);
    public static bool operator !=(Pair<T, U> left, Pair<T, U> right) => !left.Equals(right);

    public override bool Equals(object obj) => (obj is Pair<T, U> tuple) && Equals(tuple);
}
}