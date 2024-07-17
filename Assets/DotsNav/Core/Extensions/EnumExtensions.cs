using System;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using System.Runtime.InteropServices;
using static System.Runtime.CompilerServices.MethodImplOptions;
using MI = System.Runtime.CompilerServices.MethodImplAttribute;


namespace DotsNav
{
public static class EnumExtensions
{
    // Fairly certain this is not possible:
    // public static ulong AsULong<TEnum>(ref this TEnum value) where TEnum : unmanaged, IConvertible {
    //     return new EnumUnion<TEnum> { Enum = value }.ULong;
    // }

    // [StructLayout(LayoutKind.Explicit)]
    // public struct EnumUnion<TEnum> where TEnum : unmanaged, IConvertible {
    //     [FieldOffset(0)] public ulong ULong;
    //     [FieldOffset(0)] public TEnum Enum;
    // }


    const int s_ByteSize = sizeof(byte);
    const int s_UIntSize = sizeof(uint);
    const int s_ULongSize = sizeof(ulong);
    
    
    // Private Extensions:
    [MI(AggressiveInlining)] unsafe static byte* AddressAsByte<TEnum>(ref this TEnum e) where TEnum : struct, Enum => (byte*)UnsafeUtility.AddressOf(ref e);
    [MI(AggressiveInlining)] unsafe static uint* AddressAsUInt<TEnum>(ref this TEnum e) where TEnum : struct, Enum => (uint*)UnsafeUtility.AddressOf(ref e);
    [MI(AggressiveInlining)] unsafe static ulong* AddressAsULong<TEnum>(ref this TEnum e) where TEnum : struct, Enum => (ulong*)UnsafeUtility.AddressOf(ref e);
    
    [MI(AggressiveInlining)] unsafe static byte ToByte<TEnum>(ref this TEnum e) where TEnum : struct, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_ByteSize, "Enum is not a byte");
        return *e.AddressAsByte();
    }
    [MI(AggressiveInlining)] unsafe static uint ToUInt<TEnum>(ref this TEnum e) where TEnum : struct, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_UIntSize, "Enum is not an int");
        return *e.AddressAsUInt();
    }
    [MI(AggressiveInlining)] unsafe static ulong ToULong<TEnum>(ref this TEnum e) where TEnum : struct, Enum {
        Debug.Assert(UnsafeUtility.SizeOf<TEnum>() == s_ULongSize, "Enum is not a long");
        return *e.AddressAsULong();
    }
    


    // Public Extensions:
    [MI(AggressiveInlining)] public static bool IsEqualB<TEnum>(this TEnum lhs, TEnum e) where TEnum : unmanaged, Enum {
        return lhs.ToByte() == e.ToByte();
    }
    [MI(AggressiveInlining)] public static bool IsAnyEqualB<TEnum>(this TEnum lhs, TEnum e0, TEnum e1) where TEnum : unmanaged, Enum {
        byte lhsByte = lhs.ToByte();
        return lhsByte == e0.ToByte() || lhsByte == e1.ToByte();
    }
    [MI(AggressiveInlining)] public static bool IsAnyEqualB<TEnum>(this TEnum lhs, TEnum e0, TEnum e1, TEnum e2) where TEnum : unmanaged, Enum {
        byte lhsByte = lhs.ToByte();
        return lhsByte == e0.ToByte() || lhsByte == e1.ToByte() || lhsByte == e2.ToByte();
    }
    [MI(AggressiveInlining)] public static bool IsAnyEqualB<TEnum>(this TEnum lhs, TEnum e0, TEnum e1, TEnum e2, TEnum e3) where TEnum : unmanaged, Enum {
        byte lhsByte = lhs.ToByte();
        return lhsByte == e0.ToByte() || lhsByte == e1.ToByte() || lhsByte == e2.ToByte() || lhsByte == e3.ToByte();
    }

    [MI(AggressiveInlining)] public static bool IsAllEqualB<TEnum>(this TEnum lhs, TEnum e0, TEnum e1) where TEnum : unmanaged, Enum {
        byte lhsByte = lhs.ToByte();
        return lhsByte == e0.ToByte() && lhsByte == e1.ToByte();
    }
    [MI(AggressiveInlining)] public static bool IsAllEqualB<TEnum>(this TEnum lhs, TEnum e0, TEnum e1, TEnum e2) where TEnum : unmanaged, Enum {
        byte lhsByte = lhs.ToByte();
        return lhsByte == e0.ToByte() && lhsByte == e1.ToByte() && lhsByte == e2.ToByte();
    }
    [MI(AggressiveInlining)] public static bool IsAllEqualB<TEnum>(this TEnum lhs, TEnum e0, TEnum e1, TEnum e2, TEnum e3) where TEnum : unmanaged, Enum {
        byte lhsByte = lhs.ToByte();
        return lhsByte == e0.ToByte() && lhsByte == e1.ToByte() && lhsByte == e2.ToByte() && lhsByte == e3.ToByte();
    }


    // [MI(AggressiveInlining)] public static unsafe void SetFlagTest<TEnum>(ref this TEnum flags, TEnum flagToSet, bool value) where TEnum : unmanaged, Enum {
    //     EnumUnion<TEnum> flagsUnion = new EnumUnion<TEnum> { Enum = flags };
    //     flagsUnion.ULong = (flagsUnion.ULong & ~flagToSet.AsULong()) | (flagToSet.AsULong() * (ulong)(value ? 1 : 0));
    //     flags = flagsUnion.Enum;
    // }

    [MI(AggressiveInlining)] public static unsafe void SetFlagsB<TEnum>(ref this TEnum flags, TEnum flagsToSet, bool value) where TEnum : unmanaged, Enum {
        byte* flagsPtr = flags.AddressAsByte();
        *flagsPtr = (byte)(((*flagsPtr) & ~flagsToSet.ToByte()) | (flagsToSet.ToByte() * (value ? 1 : 0)));
    }


    [MI(AggressiveInlining)] public static bool HasAnyFlagsB<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        return (lhs.ToByte() & rhs.ToByte()) > 0;
    }
    [MI(AggressiveInlining)] public static bool HasAnyFlagsI<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        return (lhs.ToUInt() & rhs.ToUInt()) > 0;
    }
    [MI(AggressiveInlining)] public static bool HasAnyFlagsL<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        return (lhs.ToULong() & rhs.ToULong()) > 0;
    }

    [MI(AggressiveInlining)] public static bool HasNoFlagsB<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        return (lhs.ToByte() & rhs.ToByte()) == 0;
    }
    [MI(AggressiveInlining)] public static bool HasNoFlagsI<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        return (lhs.ToUInt() & rhs.ToUInt()) == 0;
    }
    [MI(AggressiveInlining)] public static bool HasNoFlagsL<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        return (lhs.ToULong() & rhs.ToULong()) == 0;
    }

    [MI(AggressiveInlining)] public static bool HasAllFlagsB<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        byte flags = rhs.ToByte();
        return (lhs.ToByte() & flags) == flags;
    }
    [MI(AggressiveInlining)] public static bool HasAllFlagsI<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        uint flags = rhs.ToUInt();
        return (lhs.ToUInt() & flags) == flags;
    }
    [MI(AggressiveInlining)] public static bool HasAllFlagsL<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        ulong flags = rhs.ToULong();
        return (lhs.ToULong() & flags) == flags;
    }

    [MI(AggressiveInlining)] public static bool HasOnlyFlagsB<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        byte flags = rhs.ToByte();
        return (lhs.ToByte() | flags) == flags;
    }
    [MI(AggressiveInlining)] public static bool HasOnlyFlagsI<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        uint flags = rhs.ToUInt();
        return (lhs.ToUInt() | flags) == flags;
    }
    [MI(AggressiveInlining)] public static bool HasOnlyFlagsL<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
        ulong flags = rhs.ToULong();
        return (lhs.ToULong() | flags) == flags;
    }


    // [MI(AggressiveInlining)] public static int MaskB<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
    //     return lhs.ToByte() & rhs.ToByte();
    // }
    // [MI(AggressiveInlining)] public static bool MaskI<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
    //     return (lhs.ToUInt() & rhs.ToUInt()) > 0;
    // }
    // [MI(AggressiveInlining)] public static bool MaskL<TEnum>(this TEnum lhs, TEnum rhs) where TEnum : unmanaged, Enum {
    //     return (lhs.ToULong() & rhs.ToULong()) > 0;
    // }
}
}