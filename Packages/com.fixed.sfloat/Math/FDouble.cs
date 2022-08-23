using System;
using System.Runtime.InteropServices;

namespace Morefun.LockStep
{
    [Serializable, StructLayout(LayoutKind.Sequential)]
    public struct FDouble
    {
        public const int wholeBits = 31;
        public const int fractionBits = 32;

        public long rawValue;
    }

    [Serializable, StructLayout(LayoutKind.Sequential)]
    public struct FDoubleVector3
    {
        public FDouble x;

        public FDouble y;

        public FDouble z;
    }

    [Serializable, StructLayout(LayoutKind.Sequential)]
    public struct FDoubleQuaternion
    {
        public FDouble x;

        public FDouble y;

        public FDouble z;

        public FDouble w;
    }
}
