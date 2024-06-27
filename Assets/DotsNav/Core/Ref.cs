using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using Unity.Transforms;
using Unity.Mathematics;
using Debug = UnityEngine.Debug;
using static System.Runtime.CompilerServices.MethodImplOptions;
using MI = System.Runtime.CompilerServices.MethodImplAttribute;


// TODO: Check if this is even burst compatible
// public unsafe struct Enum<T> : IEquatable<Ref<T>> where T : unmanaged, IRefable {)


namespace Unity.Collections.LowLevel.Unsafe {
public unsafe static class UnsafeLib {
    public unsafe static void ThrowIfNull(void* data) {
        if (data == null) {
            throw new NullReferenceException("NullReferenceException - Attempting to call function when data is null. Prevented a segfault");
        }
    }

    // Why would we ever need this? Should remove it
    [MI(AggressiveInlining)] public static T* UnsafeThisPtr2<T>(this ref T @struct) where T : unmanaged => PtrOf(ref @struct);

    [MI(AggressiveInlining)] public static ref T UnsafeThisRef<T>(this ref T @struct) where T : unmanaged, IRefable => ref RefOf(ref @struct);
    [MI(AggressiveInlining)] public static T* UnsafeThisPtr<T>(this ref T @struct) where T : unmanaged, IRefable => PtrOf(ref @struct);
    [MI(AggressiveInlining)] public static T* PtrOf<T>(ref T @struct) where T : unmanaged => (T*)UnsafeUtility.AddressOf<T>(ref @struct);
    [MI(AggressiveInlining)] public static ref T RefOf<T>(ref T @struct) where T : unmanaged => ref *PtrOf(ref @struct);

    [MI(AggressiveInlining)] public static long AddressAsLong<T>(T* ptr) where T : unmanaged => (long)ptr;
    [MI(AggressiveInlining)] public static T* LongAsAddress<T>(long @long) where T : unmanaged => (T*)@long;

    public static TField* GetFieldPtr<TField>(void* ptr, int fieldOffset) where TField : unmanaged {
        return (TField*) ((byte*)ptr + fieldOffset);
    }
    public static TField* GetFieldPtr<TStruct, TField>(ref TStruct @struct, int fieldOffset) where TField : unmanaged  where TStruct : unmanaged {
        byte* structAddress = (byte*)UnsafeUtility.AddressOf(ref @struct);
        return (TField*) (structAddress + fieldOffset);
    }

    public unsafe struct RefableMemory {
        int memDirtyChecker;
        const int InvalidMemory = int.MinValue + 12345678; // The following will heavily break things if (int)TStruct has this value
        public void SetMemoryDirty() => memDirtyChecker = InvalidMemory;
        public bool IsMemoryClean() => memDirtyChecker != InvalidMemory;
    }
    public static void SetMemoryDirty(void* data) { ThrowIfNull(data); ((RefableMemory*)data)->SetMemoryDirty(); } // TODO: These are temporary (not sure if will exist in future)
    public static bool IsMemoryClean(void* data) { ThrowIfNull(data); return ((RefableMemory*)data)->IsMemoryClean(); }


    public unsafe struct MoveableMemory {
        public RefableMemory refableMemory;
        public void* newAddress;
    }
    public static void* GetMovedAddress(void* data) { ThrowIfNull(data); return ((MoveableMemory*)data)->newAddress; }

    public static bool IsTypeRefable<T>() where T : unmanaged => UnsafeUtility.SizeOf<T>() >= UnsafeUtility.SizeOf<RefableMemory>();
    public static bool IsTypeMoveable<T>() where T : unmanaged => UnsafeUtility.SizeOf<T>() >= UnsafeUtility.SizeOf<MoveableMemory>();

    // public RemoveRef
}

public interface IRefable {
    // public int MemoryDirtyValue { get; } // Maybe a TODO?
    // public int IsMemoryDirty();

    public bool IsValid();
    public bool IsValidRobust() { return true; }
}
public interface IMoveable<T> where T : unmanaged {
    public unsafe void OnMoved(T* newPos); // TODO: Consider renaming to something like OnMoveFixRefs (Maybe even OnMemoryMove, to differentiate with list.Remove)
    public void OnRemoved();   
}


public unsafe static class RefableExtensions {
    public static bool IsMemoryClean<TRefable>(ref this TRefable refable) where TRefable : unmanaged, IRefable => UnsafeLib.IsMemoryClean(refable.UnsafeThisPtr());
}

public unsafe static class MoveableExtensions {
    public static void* GetMovedAddress<TMoveable, T>(ref this TMoveable moveable)
        where TMoveable : unmanaged, IMoveable<T>  where T : unmanaged => UnsafeLib.GetMovedAddress(moveable.UnsafeThisPtr2());
}


// If need a Nullable Ref just use Ptr
public unsafe readonly struct Ref<T> : IEquatable<Ref<T>>, IRefable  where T : unmanaged, IRefable { // Ref cannot be null and cannot have invalid data
    public readonly bool IsMemoryClean() => UnsafeLib.IsMemoryClean(data);
    public readonly bool IsDataValid() => IsCreated && data->IsValid(); // TODO: Maybe should throw if null?
    public bool IsCreated => data != null;

    [Conditional("UNITY_ASSERTIONS")] public void DebugAssertAccessValid() {
        if (data == null) { Debug.Assert(false, "Attempting to access when Data is null"); return; }
        Debug.Assert(IsDataValid(), "Attempting to access when Data is invalid");
        Debug.Assert(IsMemoryClean(), "Attempting to access when Memory is dirty");
    }
    [Conditional("UNITY_ASSERTIONS")] public void DebugAssertHoldingValid() {
        Debug.Assert(UnsafeLib.IsTypeRefable<T>(), "Attempting construction when this type is not Refable because it is too small");
        if (data == null) { Debug.Assert(false, "Attempting construction when Data is null"); return; }
        Debug.Assert(IsValid(), "Attempting construction when Data is invalid"); // Memory may be dirty but that is fine
    }

    readonly T* data;
    public readonly T* p { get { DebugAssertAccessValid(); return data; } } // Not allowed to access with DirtyMemory
    public readonly ref T r { get { DebugAssertAccessValid(); return ref UnsafeUtility.AsRef<T>(data); } }
    public readonly Ptr<T> AsPtr() => new Ptr<T>(this);

    public readonly T* UncheckedPtr { get { return data; } }
    
    public Ref(T* ptr) { this.data = ptr; DebugAssertHoldingValid(); } // Allowed to construct with DirtyMemory
    public Ref(IntPtr intPtr) : this((T*)intPtr.ToPointer()) {}
    public Ref(ref T refT) : this(refT.UnsafeThisPtr()) {}
    public Ref(T* ptr, bool isUnchecked) { this.data = ptr; if (!isUnchecked) { DebugAssertHoldingValid(); } }

    public bool IsValid() => IsCreated && !(IsMemoryClean() && !IsDataValid()); // Allowed to hold with DirtyMemory
    public bool IsValidRobust() => IsValid();

    public readonly bool Equals(Ref<T> other) => this == other;
    public readonly override int GetHashCode() => new IntPtr(data).GetHashCode();
    public readonly override bool Equals(object other) => this == (Ref<T>)other;

    public static bool operator ==(Ref<T> refLhs, Ref<T> refRhs) => refLhs.data == refRhs.data;
    public static bool operator !=(Ref<T> refLhs, Ref<T> refRhs) => refLhs.data != refRhs.data;

    public static bool operator ==(T* refLhsData, Ref<T> refRhs) => refLhsData == refRhs.data;
    public static bool operator !=(T* refLhsData, Ref<T> refRhs) => refLhsData != refRhs.data;
    public static bool operator ==(Ref<T> refLhs, T* refRhsData) => refLhs.data == refRhsData;
    public static bool operator !=(Ref<T> refLhs, T* refRhsData) => refLhs.data != refRhsData;

    public static implicit operator Ptr<T>(Ref<T> refT) => refT.AsPtr();
    public static implicit operator Ref<T>(T* ptrT) => new Ref<T>(ptrT);
    public static implicit operator T*(Ref<T> refT) => refT.p;
}

public unsafe readonly struct Ptr<T> : IEquatable<Ptr<T>>, IRefable  where T : unmanaged, IRefable { // Unlike Ref, Ptr is allowed to be null, but still cannot be invalid
    public static Ptr<T> Null => default;
    public readonly bool IsMemoryClean() => !(IsCreated && !UnsafeLib.IsMemoryClean(data));
    public readonly bool IsDataValid() =>  !(IsCreated && !data->IsValid());
    public readonly bool IsNull => data == null;
    public readonly bool IsCreated => data != null;

    [Conditional("UNITY_ASSERTIONS")] public void DebugAssertAccessValid() {
        Debug.Assert(IsDataValid(), "Attempting to access when Data is invalid");
        Debug.Assert(IsMemoryClean(), "Attempting to access when Memory is dirty");
    }
    [Conditional("UNITY_ASSERTIONS")] public void DebugAssertHoldingValid() {
        Debug.Assert(UnsafeLib.IsTypeRefable<T>(), "Attempting construction when this type is not Refable because it is too small");
        Debug.Assert(IsValid(), "Attempting construction when Data is invalid"); // Memory may be dirty but that is fine
    }

    readonly T* data;
    public readonly T* p { get { DebugAssertAccessValid(); return data; } } // Not allowed to access with DirtyMemory 
    public readonly ref T r { get { DebugAssertAccessValid(); return ref UnsafeUtility.AsRef<T>(data); } }
    
    public readonly T* UncheckedPtr { get { return data; } }

    public Ptr(T* ptr) { this.data = ptr; DebugAssertHoldingValid(); } // Allowed to construct with DirtyMemory
    public Ptr(IntPtr intPtr) : this((T*)intPtr.ToPointer()) {}
    public Ptr(ref T refT) : this(refT.UnsafeThisPtr()) {}
    public Ptr(Ref<T> RefT) : this(RefT.p) {}
    public Ptr(T* ptr, bool isUnchecked) { this.data = ptr; if (!isUnchecked) { DebugAssertHoldingValid(); } }

    public bool IsValid() => !(IsMemoryClean() && !IsDataValid()); // Allowed to hold with DirtyMemory
    public bool IsValidRobust() => IsValid();

    public readonly bool Equals(Ptr<T> other) => this == other;
    public readonly override int GetHashCode() => new IntPtr(data).GetHashCode();
    public readonly override bool Equals(object other) => this == (Ptr<T>)other;

    public static bool operator ==(Ptr<T> ptrLhs, Ptr<T> ptrRhs) => ptrLhs.data == ptrRhs.data;
    public static bool operator !=(Ptr<T> ptrLhs, Ptr<T> ptrRhs) => ptrLhs.data != ptrRhs.data;

    public static bool operator ==(T* ptrLhsData, Ptr<T> ptrRhs) => ptrLhsData == ptrRhs.data;
    public static bool operator !=(T* ptrLhsData, Ptr<T> ptrRhs) => ptrLhsData != ptrRhs.data;
    public static bool operator ==(Ptr<T> ptrLhs, T* ptrRhsData) => ptrLhs.data == ptrRhsData;
    public static bool operator !=(Ptr<T> ptrLhs, T* ptrRhsData) => ptrLhs.data != ptrRhsData;

    public static implicit operator Ptr<T>(T* ptrT) => new Ptr<T>(ptrT);
    public static implicit operator T*(Ptr<T> refT) => refT.p;
}
}