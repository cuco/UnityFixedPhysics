// create by devlinzhou [2016/10/11]
using System;
using System.Runtime.InteropServices;
namespace Morefun.LockStep
{
    /// <summary>
    /// Quaternion with fixed-point representation.
    /// </summary>
    [Serializable,StructLayout(LayoutKind.Sequential)]
    public struct FQuaternion : IEquatable<FQuaternion>
    {
        #region Fields

        /// <summary>
        /// The x component.
        /// </summary>
        public FScalar x;

        /// <summary>
        /// The y component.
        /// </summary>
        public FScalar y;

        /// <summary>
        /// The z component.
        /// </summary>
        public FScalar z;

        /// <summary>
        /// The w component.
        /// </summary>
        public FScalar w;

        #endregion

        #region Constants

        /// <summary>
        /// The zero quaternion.
        /// </summary>
        public static readonly FQuaternion zero     = new FQuaternion(FScalar.zero, FScalar.zero, FScalar.zero, FScalar.zero);

        /// <summary>
        /// The identity quaternion.
        /// </summary>
        public static readonly FQuaternion identity = new FQuaternion(FScalar.zero, FScalar.zero, FScalar.zero, FScalar.one );

        #endregion

        #region Properties

        /// <summary>
        /// Returns the length of this quaternion (read only).
        /// </summary>
        public FScalar magnitude
        {
            get
            {
                return FScalar.SqrtRaw(sqrMagnitudeRaw);
            }
        }

        /// <summary>
        /// Returns the squared length of this quaternion (read only).
        /// </summary>
        /// <remarks>
        /// This is faster than <c>q.magnitude</c>.
        /// </remarks>
        public FScalar sqrMagnitude
        {
            get
            {
                return Dot(this, this);
            }
        }

        /// <summary>
        /// Returns the squared length of this quaternion in raw format (read only).
        /// </summary>
        /// <returns>
        /// Returns the squared length in 44.20 format .
        /// </returns>
        public long sqrMagnitudeRaw
        {
            get
            {
                return DotRaw(this, this);
            }
        }

        /// <summary>
        /// Gets a quaternion with magnitude 1 (read only).
        /// </summary>
        /// <description>
        /// When this is a zero quaternion, it returns a zero quaternion, without throwing division-by-zero exception.
        /// </description>
        public FQuaternion normalized
        {
            get
            {
                long d = sqrMagnitudeRaw;
                if (d == 0)
                    return FQuaternion.zero;
                else
                {
                    //return this * FScalar.InvSqrtRaw(d);
                    return this / FScalar.SqrtRaw(d);
                }
            }
        }

        /// <summary>
        /// Access the x, y, z, w component using [0], [1], [2], [3] respectively.
        /// </summary>
        /// <param name="index">The index of range 0 to 3.</param>
        public FScalar this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0: return x;
                    case 1: return y;
                    case 2: return z;
                    case 3: return w;
                    default: throw new IndexOutOfRangeException();
                }
            }
            set
            {
                switch (index)
                {
                    case 0: x = value; break;
                    case 1: x = value; break;
                    case 2: x = value; break;
                    case 3: x = value; break;
                    default: throw new IndexOutOfRangeException();
                }
            }
        }

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a FQuaternion.
        /// </summary>
        /// <param name="x">The x component.</param>
        /// <param name="y">The y component.</param>
        /// <param name="z">The z component.</param>
        /// <param name="w">The w component.</param>
        public FQuaternion(FScalar x, FScalar y, FScalar z, FScalar w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }

        #endregion

        #region Public methods

        /// <summary>
        /// Makes this vector have a magnitude of 1.
        /// </summary>
        /// <description>
        /// When this is a zero vector, this is unchanged without generating division-by-zero exception.
        /// </description>
        public void Normalize()
        {
            this = normalized;
        }

        #endregion

        #region Operators

        /// <summary>
        /// Unary plus operator.
        /// </summary>
        public static FQuaternion operator+(FQuaternion q)
        {
            return q;
        }

        /// <summary>
        /// Negation operator.
        /// </summary>
        public static FQuaternion operator-(FQuaternion q)
        {
            return new FQuaternion(-q.x, -q.y, -q.z, -q.w);
        }

        /// <summary>
        /// Addition operator.
        /// </summary>
        public static FQuaternion operator+(FQuaternion a, FQuaternion b)
        {
            return new FQuaternion(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
        }

        /// <summary>
        /// Subtraction operator.
        /// </summary>
        public static FQuaternion operator-(FQuaternion a, FQuaternion b)
        {
            return new FQuaternion(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
        }

        /// <summary>
        /// Multiplication operator of two quaternion.
        /// </summary>
        public static FQuaternion operator*(FQuaternion a, FQuaternion b)
        {
            return new FQuaternion(
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
            );
        }

        /// <summary>
        /// Rotates the vector with the quaternion.
        /// </summary>
        public static FVector3 operator*(FQuaternion q, FVector3 v)
        {
            // nVidia SDK implementation
            FVector3 qVec  = new FVector3(q.x, q.y, q.z);
            FVector3 uv     = FVector3.Cross(qVec, v);
            FVector3 uuv    = FVector3.Cross(qVec, uv);
            uv  *=  2 * q.w;
            uuv *=  2;

            return v + uv + uuv;
        }

        /// <summary>
        /// Multiplication operator of a scalar and a quaternion.
        /// </summary>
        public static FQuaternion operator*(FScalar s, FQuaternion q)
        {
            return new FQuaternion(s * q.x, s * q.y, s * q.z, s * q.w);
        }

        /// <summary>
        /// Multiplication operator of a quaternion and a scalar.
        /// </summary>
        public static FQuaternion operator*(FQuaternion q, FScalar s)
        {
            return new FQuaternion(s * q.x, s * q.y, s * q.z, s * q.w);
        }

        /// <summary>
        /// Multiplication operator of a quaternion and a scalar.
        /// </summary>
        public static FQuaternion operator/(FQuaternion q, FScalar s)
        {
            return new FQuaternion(q.x / s, q.y / s, q.z / s, q.w / s);
        }

        /// <summary>
        /// Multiplication operator of a scalar and a quaternion.
        /// </summary>
        public static FQuaternion operator/(FScalar s, FQuaternion q)
        {
            return new FQuaternion(s / q.x, s / q.y, s / q.z, s / q.w);
        }

        /// <summary>
        /// Equality operator.
        /// </summary>
        public static bool operator==(FQuaternion a, FQuaternion b)
        {
            return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
        }

        /// <summary>
        /// Inequality operator.
        /// </summary>
        public static bool operator!=(FQuaternion a, FQuaternion b)
        {
            return a.x != b.x || a.y != b.y || a.z != b.z || a.w != b.w;
        }

        #endregion

        #region Static Methods

        public static FScalar Angle(FQuaternion a, FQuaternion b)
        {
            // 修改以与Unity保持一致
            return FScalar.Acos(FScalar.Min(FScalar.Abs(Dot(a, b)), FScalar.one)) * 2;
        }

        public static FQuaternion AngleAxis(FScalar angle, FVector3 axis)
        {
            FScalar s, c;
            FScalar.SinCos(angle / 2, out s, out c);
            return new FQuaternion(s * axis.x, s * axis.y, s * axis.z, c);
        }

        /// <summary>
        /// Dot product of two quaternion.
        /// </summary>
        /// <param name="a">The first quaternion.</param>
        /// <param name="b">The second quaternion.</param>
        /// <remarks>
        /// Dot product is commutative: <c></c>FQuaternion.Dot(a, b) == FQuaternion.Dot(b, a)</c>
        /// </remarks>
        public static FScalar Dot(FQuaternion a, FQuaternion b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
        }

        /// <summary>
        /// Dot product of two quaternions in raw format.
        /// </summary>
        /// <param name="a">The first quaternion.</param>
        /// <param name="b">The second quaternion.</param>
        /// <returns>Dot product in 44.20 raw format</returns>
        /// <remarks>
        /// Dot product is commutative: <c></c>FQuaternion.Dot(a, b) == FQuaternion.Dot(b, a)</c>
        /// </remarks>
        public static long DotRaw(FQuaternion a, FQuaternion b)
        {
            return  
                (long)a.x.rawValue * (long)b.x.rawValue + 
                (long)a.y.rawValue * (long)b.y.rawValue +
                (long)a.z.rawValue * (long)b.z.rawValue +
                (long)a.w.rawValue * (long)b.w.rawValue;
        }
        
        public static FQuaternion CreateQuaternionFromAxisQuaternions(FQuaternion q1, FQuaternion q2, FQuaternion q3)
        {
            var result = (q1 * q2) * q3;
            //Assert(CompareApproximately(SqrMagnitude(result), 1.0F));
            return result;
        }

        //参考Unity的vec-quat.h eulerToQuat quatToEuler
        //TODO 未测试
        //Unity:Quaternion.cpp EulerToQuaternion
        // default: ZXY
        /// <summary>
        /// 
        /// </summary>
        /// <param name="euler">欧拉角，单位度数</param>
        /// <returns></returns>
        public static FQuaternion Euler(FVector3 euler)
        {
            euler /= FScalar.deg360; 
            // x roll. y pitch. z yaw.
            FScalar.SinCos(euler.x / 2, out var sx, out var cx);
            FScalar.SinCos(euler.y / 2, out var sy, out var cy);
            FScalar.SinCos(euler.z / 2, out var sz, out var cz);
            
            FQuaternion qX = new FQuaternion(sx, 0, 0, cx);
            FQuaternion qY = new FQuaternion(0, sy, 0, cy);
            FQuaternion qZ = new FQuaternion(0, 0, sz, cz);
            FQuaternion ret = CreateQuaternionFromAxisQuaternions(qY, qX, qZ);
    
            ret.Normalize();
            // return new FQuaternion(
            //     sx * cz * cz - cx * sz * sz,
            //     cx * sz * cz + sx * cz * sz,
            //     cx * cz * sz - sx * sz * cz,
            //     cx * cz * cz + sx * sz * sz);
            return ret;
        }
         
        public static FQuaternion FromToRotation(FVector3 fromDirection, FVector3 toDirection )
        {
            FVector3 TUP = FVector3.Cross(fromDirection, toDirection);
            if( TUP.magnitude < FScalar.FromRaw(3))
            {
                //TODO: 寻找更好的解决方案处理从该坐标轴旋转180度的情况
                if (fromDirection == FVector3.forward)
                {
                    return AngleAxis(
                        FVector3.Angle(fromDirection, toDirection),
                        FVector3.up);
                }
                else
                {
                    return AngleAxis(
                        FVector3.Angle(fromDirection, toDirection),
                        FVector3.forward);
                }
            }
            else
            {
                return AngleAxis(
                FVector3.Angle(fromDirection, toDirection),
                FVector3.Cross(fromDirection, toDirection).normalized);
            }
        }

        public static FQuaternion Conjugate(FQuaternion q)
        {
            return new FQuaternion(-q.x, -q.y, -q.z, q.w);
        }

        public static FQuaternion Inverse(FQuaternion q)
        {
            return Conjugate(q) / q.sqrMagnitude;
        }

        public static FQuaternion Lerp(FQuaternion a, FQuaternion b, FScalar t)
        {
            t = FScalar.Saturate(t);
            return ((FScalar.one - t) * a + t * b).normalized;
        }

        public static FQuaternion LerpUnclapmed(FQuaternion a, FQuaternion b, FScalar t)
        {
            return ((FScalar.one - t) * a + t * b).normalized;
        }

        public static FQuaternion Slerp(FQuaternion a, FQuaternion b, FScalar t)
        {
            FScalar cosomega = Dot(a, b);
            FScalar abscosomega = FScalar.Abs(cosomega);
            FScalar scale0, scale1;
            if (FScalar.one - abscosomega > FScalar.epsilon * 4) {
                FScalar sinomegasq = FScalar.one - abscosomega * abscosomega;
                FScalar sinomega = FScalar.InvSqrt(sinomegasq);
                FScalar omega = FScalar.Atan2(sinomegasq * sinomega, abscosomega);
                scale0 = FScalar.Sin((FScalar.one - t) * omega) * sinomega;
                scale1 = FScalar.Sin(t * omega) * sinomega;
            }
            else {
                scale0 = FScalar.one - t;
                scale1 = t;
            }
            scale1 = (cosomega >= FScalar.zero) ? scale1 : -scale1;
            return a * scale0 + b * scale1;
        }

        public static FQuaternion SlerpUnclamped(FQuaternion a, FQuaternion b, FScalar t)
        {
            return Slerp(a, b, FScalar.Saturate(t));
        }

        //enum QuatIndexes
        // {
        //     xx = 0,
        //     xy = 1,
        //     xz = 2,
        //     xw = 3,
        //     yy = 4,
        //     yz = 5,
        //     yw = 6,
        //     zz = 7,
        //     zw = 8,
        //     ww = 9,
        //     QuatIndexesCount
        // };
        
        // enum Indexes
        // {
        //     X1 = 0,
        //     X2 = 1,
        //     Y1 = 2,
        //     Y2 = 3,
        //     Z1 = 4,
        //     Z2 = 5,
        //     singularity_test = 6,
        //     IndexesCount
        // };
        /// <summary>
        /// 返回的是turn （0，1）
        /// TODO 优化
        /// </summary>
        /// <param name="q"></param>
        /// <returns></returns>
        private static FVector3 QuaternionToEuler(FQuaternion q)
        {
            //setup all needed values
            FScalar[] d = {
                q.x * q.x,
                q.x * q.y, 
                q.x * q.z, 
                q.x * q.w, 
                q.y * q.y, 
                q.y * q.z, 
                q.y * q.w, 
                q.z * q.z, 
                q.z * q.w, 
                q.w * q.w};
            FScalar[] v = new FScalar[7];
            //f {&qAsin, &qAtan2, &qAtan2},
            FVector3 rot;
            
            v[6] = d[5] - d[3];
            v[4] = 2 * (d[1] + d[8]);
            v[5] = d[4] - d[7] - d[0] + d[9];
            v[0] = -1;
            v[1] = 2 * v[6];

            if (FScalar.Abs(v[6]) < FScalar.half)
            {
                v[2] = 2 * (d[2] + d[6]);
                v[3] = d[7] - d[0] - d[4] + d[9];
                
                rot = new FVector3(v[0] * FScalar.Asin(FScalar.Clamp(v[1],-1,1)),
                    FScalar.Atan2(v[2], v[3]),
                    FScalar.Atan2(v[4], v[5]));
            }
            else //x == yzy z == 0
            {
                FScalar a, b, c, e;
                a = d[1] + d[8];
                b = -d[5] + d[3];
                c = d[1] - d[8];
                e = d[5] + d[3];

                v[2] = a * e + b * c;
                v[3] = b * e - a * c;
                rot = new FVector3(v[0] * FScalar.Asin(FScalar.Clamp(v[1],-1,1)),
                    FScalar.Atan2(v[2], v[3]),
                    0);
            }

            return rot;
        }
        
        /// <summary>
        /// 传入的是角度，返回的也是角度
        /// </summary>
        /// <param name="euler"></param>
        /// <returns></returns>
        private static FVector3 MakePositive(FVector3 euler)
        {
            FScalar num1 = -9 / (500 * FScalar.pi);
            FScalar num2 = FScalar.deg360 + num1;
            if (euler.x < num1)
                euler.x += FScalar.deg360;
            else if (euler.x > num2)
                euler.x -= FScalar.deg360;
            if (euler.y < num1)
                euler.y += FScalar.deg360;
            else if (euler.y > num2)
                euler.y -= FScalar.deg360;
            if (euler.z < num1)
                euler.z += FScalar.deg360;
            else if (euler.z > num2)
                euler.z -= FScalar.deg360;
            return euler;
        }

        private static readonly FScalar s_magicNum = new FScalar(57, 2957, 10000);
        //Unity default rotation order. Extrinsic Rotation around the z axis, then around the x axis and finally around the y axis.
        // 参考：
        // Quaternion.cs eulerAngles
        // Quaternion.cpp QuaternionToEuler
        /// <summary>
        /// 返回欧拉角，度数
        /// </summary>
        /// <param name="rotation"></param>
        /// <returns></returns>
        public static FVector3 ToEulerAngles(FQuaternion q)
        {
            var ret = QuaternionToEuler(q) * 2 * FScalar.pi;
            ret = MakePositive(ret * s_magicNum);
            return ret;
        }
        
        public static bool IsUnit (FQuaternion q) {
            return (1 - q.w * q.w - q.x * q.x - q.y * q.y - q.z * q.z) <= FScalar.epsilon;
        }

        #endregion

        #region IEquatable<FVector2>
        
        public bool Equals(FQuaternion rhs)
        {
            return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
        }

        #endregion

        #region Object
        
        public override bool Equals(object obj)
        {
            return obj is FQuaternion && ((FQuaternion)obj) == this;
        }
        
        public override int GetHashCode()
        {
            int hash = 17;
            hash = hash * 31 + x.GetHashCode();
            hash = hash * 31 + y.GetHashCode();
            hash = hash * 31 + z.GetHashCode();
            hash = hash * 31 + w.GetHashCode();
            return hash;
        }

        public override string ToString()
        {
            return x + ", " + y + ", " + z + ", " + w;
        }

        #endregion
    }
}
