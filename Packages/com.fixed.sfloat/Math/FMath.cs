using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using System.Diagnostics;

namespace Morefun.LockStep
{
    public interface IFMath
    {
        int Sqrt(int x);
        int SqrtRaw(long x);
    }

    public class FMath
    {
        static IFMath m_FMath = null;

        public static void SetMath(IFMath TMath)
        {
            m_FMath = TMath;
        }

        public static int Clamp(int value, int min, int max)
        {
            return Math.Min(Math.Max(value, min), max);
        }

        public static long Clamp(long value, long min, long max)
        {
            return Math.Min(Math.Max(value, min), max);
        }

        /// <summary>
        /// Square root.
        /// </summary>
        /// <remarks>
        /// Turkowski, Ken. "Fixed point square root." Apple Computer, Inc., Tech. Rep 96 (1994).
        /// http://www.realitypixels.com/turk/computergraphics/FixedSqrt.pdf
        /// </remarks>
        internal static int Sqrt(int x)
        {
            LockstepDebug.Assert(x >= 0);

            if (m_FMath != null)
            {
                LockstepDebug.CheckCSAndCPP(m_FMath.Sqrt(x) == Sqrt_CS(x));

                return m_FMath.Sqrt(x);
            }

            return Sqrt_CS(x);
        }

        internal static int Sqrt_CS(int x)
        {
            uint root = 0; /* Clear root */
            uint remHi = 0; /* Clear high part of partial remainder */
            uint remLo = (uint) x; /* Get argument into low part of partial remainder */
            uint count = 15 + (FScalar.fractionBits >> 1); /* Load loop counter */
            do
            {
                remHi = (remHi << 2) | (remLo >> 30);
                remLo <<= 2; /* get 2 bits of arg */
                root <<= 1; /* Get ready for the next bit in the root */
                uint testDiv = (root << 1) + 1; /* Test radical */
                if (remHi >= testDiv)
                {
                    remHi -= testDiv;
                    root++;
                }
            } while (count-- != 0);

            return (int) root;
        }

        /// <summary>
        /// Square root of x, which is multiplication of two FScalar in raw values.
        /// </summary>
        /// <description>
        /// This function is for preventing overflow ehn compuating magnitude of big vectors, and similar problems.
        /// </description>
        /// <code>
        /// FScalar a = ..., b = ...;
        /// long x = (long)a.raw * (long)b.raw;
        /// FScalar c = SqrtRaw(x);
        /// </code>
        /// </remarks>
        internal static int SqrtRaw(long x)
        {
            LockstepDebug.Assert(x >= 0);

            if (m_FMath != null)
            {
                LockstepDebug.CheckCSAndCPP(m_FMath.SqrtRaw(x) == SqrtRaw_CS(x));

                return m_FMath.SqrtRaw(x);
            }

            return SqrtRaw_CS(x);
        }

        internal static int SqrtRaw_CS(long x)
        {
            LockstepDebug.Assert(x >= 0);

            if (m_FMath != null)
            {
                return m_FMath.SqrtRaw(x);
            }

            ulong root = 0; /* Clear root */
            ulong remHi = 0; /* Clear high part of partial remainder */
            ulong remLo = (ulong) x; /* Get argument into low part of partial remainder */
            uint count = 31; /* Load loop counter */
            do
            {
                remHi = (remHi << 2) | (remLo >> 62);
                remLo <<= 2; /* get 2 bits of arg */
                root <<= 1; /* Get ready for the next bit in the root */
                ulong testDiv = (root << 1) + 1; /* Test radical */
                if (remHi >= testDiv)
                {
                    remHi -= testDiv;
                    root++;
                }
            } while (count-- != 0);

            return (int) root;
        }

        public static FMatrix3x3 FQuaternionToFMatrix3x3(FQuaternion q)
        {
            FScalar d = q.sqrMagnitude;
            FScalar s = Sqrt(2) / d;

            FScalar xs = q.x * s, ys = q.y * s, zs = q.z * s;
            FScalar wx = q.w * xs, wy = q.w * ys, wz = q.w * zs;
            FScalar xx = q.x * xs, xy = q.x * ys, xz = q.x * zs;
            FScalar yy = q.y * ys, yz = q.y * zs, zz = q.z * zs;
            FMatrix3x3 ret = new FMatrix3x3(
                FScalar.one - (yy + zz), xy - wz, xz + wy,
                xy + wz, FScalar.one - (xx + zz), yz - wx,
                xz - wy, yz + wx, FScalar.one - (xx + yy));
            return ret;
        }

        public static FQuaternion FMatrix3x3ToFQuaternion(FMatrix3x3 mat)
        {
            FScalar t;
            FScalar[] q = new FScalar[4];

            if (mat.m22 < 0)
            {
                if (mat.m00 > mat.m11)
                {
                    t = 1 + mat.m00 - mat.m11 - mat.m22;
                    q[0] = t;
                    q[1] = mat.m10 + mat.m01;
                    q[2] = mat.m02 + mat.m20;
                    q[3] = mat.m21 - mat.m12;
                }
                else
                {
                    t = 1 - mat.m00 + mat.m11 - mat.m22;
                    q[0] = mat.m10 + mat.m01;
                    q[1] = t;
                    q[2] = mat.m21 + mat.m12;
                    q[3] = mat.m02 - mat.m20;
                }
            }
            else
            {
                if (mat.m00 < -mat.m11)
                {
                    t = 1 - mat.m00 - mat.m11 + mat.m22;
                    q[0] = mat.m02 + mat.m20;
                    q[1] = mat.m21 + mat.m12;
                    q[2] = t;
                    q[3] = mat.m10 - mat.m01;
                }
                else
                {
                    t = 1 + mat.m00 + mat.m11 + mat.m22;
                    q[0] = mat.m21 - mat.m12;
                    q[1] = mat.m02 - mat.m20;
                    q[2] = mat.m10 - mat.m01;
                    q[3] = t;
                }
            }

            FScalar halfinvt = FScalar.InvSqrt(t) / 2;
            return new FQuaternion(q[0] * halfinvt, q[1] * halfinvt, q[2] * halfinvt, q[3] * halfinvt);
        }

        /// <summary>
        /// 根据输入的向量和法线对向量进行分解
        /// </summary>
        public static void DecomposeVector(
            out FVector3 normalCompo,
            out FVector3 tangentCompo,
            FVector3 outwardDir,
            FVector3 outwardNormal)
        {
            normalCompo = outwardNormal * FVector3.Dot(outwardDir, outwardNormal);
            tangentCompo = outwardDir - normalCompo;
        }

        /// <summary>
        /// 判断一个向量是不是约等于 0
        /// </summary>
        public static bool IsAlmostZero(FVector3 v)
        {
            return FScalar.Abs(v.x) <= FScalar.epsilon
                   && FScalar.Abs(v.y) <= FScalar.epsilon
                   && FScalar.Abs(v.z) <= FScalar.epsilon;
        }

        /// <summary>
        /// 计算反射向量
        /// </summary>
        public static FVector3 ComputeReflexionVector(FVector3 incomingDir, FVector3 outwardNormal)
        {
            return FVector3.Reflect(incomingDir, outwardNormal);
        }

        /// <summary>
        /// 将整数与小数相乘返回整数
        /// </summary>
        /// <returns></returns>
        public static int MutiplyFscalarInt(int a, FScalar b)
        {
            int ret = (int)(((long) b.rawValue * a) >> FScalar.fractionBits);
            return ret;
        }
    }

    internal static class Int32Extensions
    {
        // http://msdn.microsoft.com/en-us/library/system.object.gethashcode(v=vs.110).aspx
        public static Int32 ShiftAndWrap(this Int32 value, Int32 positions = 2)
        {
            positions = positions & 0x1F;
            uint number = BitConverter.ToUInt32(BitConverter.GetBytes(value), 0);
            uint wrapped = number >> (32 - positions);
            return BitConverter.ToInt32(BitConverter.GetBytes((number << positions) | wrapped), 0);
        }
    }
}