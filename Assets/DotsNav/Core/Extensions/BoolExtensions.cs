using System.Runtime.InteropServices;

namespace DotsNav
{

    public static class BoolExtensions
    {
        public static byte AsByte(this bool value) {
            return new BoolUnion { Bool = value }.Byte;
        }

        [StructLayout(LayoutKind.Explicit)]
        private struct BoolUnion {
            [FieldOffset(0)] public bool Bool;
            [FieldOffset(0)] public byte Byte;
        }
    }
}
