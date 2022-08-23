using System;
using System.Runtime.InteropServices;

namespace Morefun.LockStep
{
    [StructLayout (LayoutKind.Sequential), Serializable]
    public struct FMatrix3x3 : IEquatable<FMatrix3x3>
    {
        //       col0 col1 col2
        //  ----+--------------
        // row0 | m00, m01, m02
        // row1 | m10, m11, m12
        // row2 | m20, m21, m22
        
        public static readonly FMatrix3x3 Identity = new FMatrix3x3(
            1, 0, 0,
            0, 1, 0,
            0, 0, 1);

        public FScalar m00, m01, m02;
        public FScalar m10, m11, m12;
        public FScalar m20, m21, m22;

        public FMatrix3x3(
            FScalar m00, FScalar m01, FScalar m02,
            FScalar m10, FScalar m11, FScalar m12,
            FScalar m20, FScalar m21, FScalar m22)
        {
            this.m00 = m00; this.m01 = m01; this.m02 = m02;
            this.m10 = m10; this.m11 = m11; this.m12 = m12;
            this.m20 = m20; this.m21 = m21; this.m22 = m22;
        }

        //?
        public FMatrix3x3(
            FVector3 col0,
            FVector3 col1,
            FVector3 col2)
        {
            this.m00 = col0.x; this.m01 = col1.x; this.m02 = col2.x;
            this.m10 = col0.y; this.m11 = col1.y; this.m12 = col2.y;
            this.m20 = col0.z; this.m21 = col1.z; this.m22 = col2.z;
        }

        public FScalar this[int row, int col]
        {
            get
            {
                if (row == 0 && col == 0)
                    return m00;
                if (row == 0 && col == 1)
                    return m01;
                if (row == 0 && col == 2)
                    return m02;
                if (row == 1 && col == 0)
                    return m10;
                if (row == 1 && col == 1)
                    return m11;
                if (row == 1 && col == 2)
                    return m12;
                if (row == 2 && col == 0)
                    return m20;
                if (row == 2 && col == 1)
                    return m21;

                return m22;
            }

            set
            {
                if (row == 0 && col == 0)
                    m00 = value;
                if (row == 0 && col == 1)
                    m01 = value;
                if (row == 0 && col == 2)
                    m02 = value;
                if (row == 1 && col == 0)
                    m10 = value;
                if (row == 1 && col == 1)
                    m11 = value;
                if (row == 1 && col == 2)
                    m12 = value;
                if (row == 2 && col == 0)
                    m20 = value;
                if (row == 2 && col == 1)
                    m21 = value;
                if (row == 2 && col == 2)
                    m22 = value;
            }
        }

        public FScalar Get(int row, int col)
        {
            if (row == 0 && col == 0)
                return m00;
            if (row == 0 && col == 1)
                return m01;
            if (row == 0 && col == 2)
                return m02;
            if (row == 1 && col == 0)
                return m10;
            if (row == 1 && col == 1)
                return m11;
            if (row == 1 && col == 2)
                return m12;
            if (row == 2 && col == 0)
                return m20;
            if (row == 2 && col == 1)
                return m21;

            return m22;
        }

        /// <summary>
        /// 获取第一列向量
        /// </summary>
        public FVector3 Col0
        {
            get => new FVector3(m00, m10, m20);

            set
            {
                m00 = value.x; 
                m10 = value.y; 
                m20 = value.z;
            }
        }

        /// <summary>
        /// 获取第二列向量
        /// </summary>
        public FVector3 Col1
        {
            get => new FVector3(m01, m11, m21);

            set
            {
                m01 = value.x;
                m11 = value.y;
                m21 = value.z;
            }
        }

        /// <summary>
        /// 获取第三列向量
        /// </summary>
        public FVector3 Col2
        {
            get => new FVector3(m02, m12, m22);

            set
            {
                m02 = value.x;
                m12 = value.y;
                m22 = value.z;
            }
        }

        /// <summary>
        /// 获取第一行向量
        /// </summary>
        public FVector3 Row0
        {
            get => new FVector3(m00, m01, m02);

            set
            {
                m00 = value.x; m01 = value.y; m02 = value.z;
            }
        }

        /// <summary>
        /// 获取第二行向量
        /// </summary>
        public FVector3 Row1
        {
            get => new FVector3(m10, m11, m12);

            set
            {
                m10 = value.x; m11 = value.y; m12 = value.z;
            }
        }

        /// <summary>
        /// 获取第三行向量
        /// </summary>
        public FVector3 Row2
        {
            get => new FVector3(m20, m21, m22);

            set
            {
                m20 = value.x; m21 = value.y; m22 = value.z;
            }
        }

        /// <summary>
        /// 重写反射默认函数
        /// </summary>
        /// <param name="obj"></param>
        /// <returns></returns>
        public override bool Equals(object obj)
        {
            return (obj is FMatrix3x3 other) && Equals(other);
        }

        public bool Equals(FMatrix3x3 other)
        {
            return this == other;
        }

        public override int GetHashCode()
        {
            return m00.GetHashCode()                 ^ m01.GetHashCode().ShiftAndWrap(2) 
                 ^ m02.GetHashCode().ShiftAndWrap(4) ^ m10.GetHashCode().ShiftAndWrap(6) 
                 ^ m11.GetHashCode().ShiftAndWrap(8) ^ m12.GetHashCode().ShiftAndWrap(10)
                 ^ m20.GetHashCode().ShiftAndWrap(12)^ m21.GetHashCode().ShiftAndWrap(14) 
                 ^ m22.GetHashCode().ShiftAndWrap(16);
        }

        public override string ToString()
        {
            return $"m00:{m00}, m01:{m01}, m02:{m02}," 
                  +$"m10:{m10}, m11:{m11}, m12:{m12}," 
                  +$"m20:{m20}, m21:{m11}, m22:{m22},";
        }

        /// <summary>
        /// 重载运算符
        /// </summary>
        public static bool operator ==(FMatrix3x3 a, FMatrix3x3 b)
        {
            return (a.m00 == b.m00 && a.m01 == b.m01 && a.m02 == b.m02 
                 && a.m10 == b.m10 && a.m11 == b.m11 && a.m12 == b.m12 
                 && a.m20 == b.m20 && a.m21 == b.m21 && a.m22 == b.m22);
        }

        public static bool operator !=(FMatrix3x3 a, FMatrix3x3 b)
        {
            return !(a == b);
        }

        public static FMatrix3x3 operator +(FMatrix3x3 a, FMatrix3x3 b)
        {
            return new FMatrix3x3(
                a.m00 + b.m00, a.m01 + b.m01, a.m02 + b.m02,
                a.m10 + b.m10, a.m11 + b.m11, a.m12 + b.m12,
                a.m20 + b.m20, a.m21 + b.m21, a.m22 + b.m22);
        }
        public static FMatrix3x3 operator -(FMatrix3x3 a, FMatrix3x3 b)
        {
            return new FMatrix3x3(
                a.m00 - b.m00, a.m01 - b.m01, a.m02 - b.m02,
                a.m10 - b.m10, a.m11 - b.m11, a.m12 - b.m12,
                a.m20 - b.m20, a.m21 - b.m21, a.m22 - b.m22);
        }

        public static FMatrix3x3 operator -(FMatrix3x3 a)
        {
            return new FMatrix3x3(
                -a.m00, -a.m01, -a.m02,
                -a.m10, -a.m11, -a.m12,
                -a.m20, -a.m21, -a.m22
                );
        }

        public static FMatrix3x3 operator *(FMatrix3x3 a, FScalar len)
        {
            return new FMatrix3x3(
                a.m00 * len, a.m01 * len, a.m02 * len,
                a.m10 * len, a.m11 * len, a.m12 * len,
                a.m20 * len, a.m21 * len, a.m22 * len);
        }

        public static FVector3 operator *(FMatrix3x3 a, FVector3 vec)
        {
            return new FVector3(
                (a.m00 * vec.x + a.m01 * vec.y + a.m02 * vec.z),
                (a.m10 * vec.x + a.m11 * vec.y + a.m12 * vec.z),
                (a.m20 * vec.x + a.m21 * vec.y + a.m22 * vec.z)
            );
        }

        public static FMatrix3x3 operator *(FMatrix3x3 a, FMatrix3x3 b)
        {
            return new FMatrix3x3(
                (a.m00 * b.m00 + a.m01 * b.m10 + a.m02 * b.m20),
                (a.m00 * b.m01 + a.m01 * b.m11 + a.m02 * b.m21),
                (a.m00 * b.m02 + a.m01 * b.m12 + a.m02 * b.m22),
                (a.m10 * b.m00 + a.m11 * b.m10 + a.m12 * b.m20),
                (a.m10 * b.m01 + a.m11 * b.m11 + a.m12 * b.m21),
                (a.m10 * b.m02 + a.m11 * b.m12 + a.m12 * b.m22),
                (a.m20 * b.m00 + a.m21 * b.m10 + a.m22 * b.m20),
                (a.m20 * b.m01 + a.m21 * b.m11 + a.m22 * b.m21),
                (a.m20 * b.m02 + a.m21 * b.m12 + a.m22 * b.m22)
            );
        }

        /// <summary>
        /// 静态函数
        /// </summary>
        /// <param name=""></param>
        /// <returns></returns>
        /// 
        public FMatrix3x3 CreateRotationMatrixFromEulerX(FScalar Xturn)
        {
            FMatrix3x3 rot = new FMatrix3x3();
            FScalar sinX = FScalar.Sin(Xturn);
            FScalar cosX = FScalar.Cos(Xturn);

            return rot;
        }
        public FMatrix3x3 CreateRotationMatrixFromEulerZXY(FVector3 euler)
        {
            FMatrix3x3 rot = new FMatrix3x3();



            return rot;
        }
    }
}