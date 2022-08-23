using System;
using System.Runtime.InteropServices;

namespace Morefun.LockStep
{
    /// <summary>
    /// 3D vector fixed-point representation.
    /// </summary>
    [Serializable, StructLayout(LayoutKind.Sequential)]
    public struct FVector3 : IEquatable<FVector3>
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

        #endregion

        #region Constants

        /// <summary>
        /// The zero vector.
        /// </summary>
        public static readonly FVector3 zero = new FVector3(FScalar.zero, FScalar.zero, FScalar.zero);

        /// <summary>
        /// The vector of (1, 1).
        /// </summary>
        public static readonly FVector3 one = new FVector3(FScalar.one, FScalar.one, FScalar.one);

        /// <summary>
        /// The vector of minimum components.
        /// </summary>
        public static readonly FVector3 min = new FVector3(FScalar.minValue, FScalar.minValue, FScalar.minValue);

        /// <summary>
        /// The vector of maximum components.
        /// </summary>
        public static readonly FVector3 max = new FVector3(FScalar.maxValue, FScalar.maxValue, FScalar.maxValue);

        /// <summary>
        /// The back direction (0, 0, -1).
        /// </summary>
        public static readonly FVector3 back = new FVector3(FScalar.zero, FScalar.zero, -FScalar.one);

        /// <summary>
        /// The left direction (-1, 0, 0).
        /// </summary>
        public static readonly FVector3 left = new FVector3(-FScalar.one, FScalar.zero, FScalar.zero);

        /// <summary>
        /// The right direction (1, 0, 0).
        /// </summary>
        public static readonly FVector3 right = new FVector3(FScalar.one, FScalar.zero, FScalar.zero);

        /// <summary>
        /// The forward direction (0, 0, 1).
        /// </summary>
        public static readonly FVector3 forward = new FVector3(FScalar.zero, FScalar.zero, FScalar.one);

        /// <summary>
        /// The up direction (0, 1, 0).
        /// </summary>
        public static readonly FVector3 up = new FVector3(FScalar.zero, FScalar.one, FScalar.zero);

        /// <summary>
        /// The up direction (0, -1, 0).
        /// </summary>
        public static readonly FVector3 down = new FVector3(FScalar.zero, -FScalar.one, FScalar.zero);

        #endregion

        #region Properties

        /// <summary>
        /// Returns the length of this vector (read only).
        /// </summary>
        public FScalar magnitude
        {
            get
            {
                return FScalar.SqrtRaw(sqrMagnitudeRaw);
            }
        }

        /// <summary>
        /// Returns the squared length of this vector (read only).
        /// </summary>
        /// <remarks>
        /// This is faster than <c>v.magnitude</c>.
        /// </remarks>
        public FScalar sqrMagnitude
        {
            get
            {
                return Dot(this, this);
            }
        }

        /// <summary>
        /// Returns the squared length of this vector in raw format (read only).
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
        /// Gets a vector of same direction with magnitude 1 (read only).
        /// </summary>
        /// <description>
        /// When this is a zero vector, this is return a zero vector, without generating division-by-zero exception.
        /// </description>
        public FVector3 normalized
        {
            get
            {
                long d = sqrMagnitudeRaw;
                if (d == 0)
                    return FVector3.zero;
                else
                    return this / FScalar.SqrtRaw(d);
            }
        }

        /// <summary>
        /// Access the x or y component using [0] or [1] respectively.
        /// </summary>
        /// <param name="index">The index of either 0 or 1.</param>
        public FScalar this[int index]
        {
            get
            {
                switch (index) {
                    case 0: return x;
                    case 1: return y;
                    case 2: return z;
                    default: throw new IndexOutOfRangeException();
                }
            }
            set
            {
                switch (index) {
                    case 0: x = value; break;
                    case 1: y = value; break;
                    case 2: z = value; break;
                    default: throw new IndexOutOfRangeException();
                }
            }
        }

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a FVector3.
        /// </summary>
        /// <param name="x">The x component.</param>
        /// <param name="y">The y component.</param>
        /// <param name="z">The z component.</param>
        public FVector3(FScalar x, FScalar y, FScalar z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public FVector3(int[] args)
        {
            x = 0;
            y = 0;
            z = 0;
            if (args != null && args.Length >= 3)
            {
                this.x = args[0];
                this.y = args[1];
                this.z = args[2];
            }
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

        public FVector2 ToVector2()
        {
            return new FVector2(x, y);
        }
        
        public FVector2 ToVector2XZ()
        {
            return new FVector2(x, z);
        }
        
        public FVector3 ToPlaneVector3()
        {
            return new FVector3(x, 0, z);
        }

        #endregion

        #region Operators

        /// <summary>
        /// Unary plus operator.
        /// </summary>
        public static FVector3 operator+(FVector3 v)
        {
            return v;
        }

        /// <summary>
        /// Negation operator.
        /// </summary>
        public static FVector3 operator-(FVector3 v)
        {
            return new FVector3(-v.x, -v.y, -v.z);
        }

        /// <summary>
        /// Addition operator.
        /// </summary>
        public static FVector3 operator+(FVector3 a, FVector3 b)
        {
            return new FVector3(a.x + b.x, a.y + b.y, a.z + b.z);
        }

        /// <summary>
        /// Subtraction operator.
        /// </summary>
        public static FVector3 operator-(FVector3 a, FVector3 b)
        {
            return new FVector3(a.x - b.x, a.y - b.y, a.z - b.z);
        }

        /// <summary>
        /// Multiplication operator of a vector and a scalar.
        /// </summary>
        public static FVector3 operator*(FVector3 v, FScalar s)
        {
            return new FVector3(v.x * s, v.y * s, v.z * s);
        }

        /// <summary>
        /// Multiplication operator of a vector and a scalar (as int).
        /// </summary>
        /// <remarks>
        /// Slightly faster than <c>operator*(FVector3 v, FScalar s)</c>.
        /// </remarks>
        public static FVector3 operator*(FVector3 v, int s)
        {
            return new FVector3(v.x * s, v.y * s, v.z * s);
        }

        /// <summary>
        /// Multiplication operator of a scalar and a vector.
        /// </summary>
        public static FVector3 operator*(FScalar s, FVector3 v)
        {
            return new FVector3(s * v.x, s * v.y, s * v.z);
        }

        /// <summary>
        /// Multiplication operator of a scalar (as <c>int</c>) and a vector.
        /// </summary>
        /// <remarks>
        /// Slightly faster than <c>operator*(FScalar s, FVector2 v)</c>.
        /// </remarks>
        public static FVector3 operator*(int s, FVector3 v)
        {
            return new FVector3(s * v.x, s * v.y, s * v.z);
        }

        /// <summary>
        /// Division operator of a vector by a scalar.
        /// </summary>
        public static FVector3 operator/(FVector3 v, FScalar s)
        {
            return new FVector3(v.x / s, v.y / s, v.z / s);
        }

        /// <summary>
        /// Division operator of a vector by a scalar (as <c>int</c>).
        /// </summary>
        /// <remarks>
        /// Slightly faster than <c>operator/(FVector2 v, FScalar s)</c>.
        /// </remarks>
        public static FVector3 operator/(FVector3 v, int s)
        {
            return new FVector3(v.x / s, v.y / s, v.z / s);
        }

        /// <summary>
        /// Division operator of a scalar by a vector.
        /// </summary>
        public static FVector3 operator/(FScalar s, FVector3 v)
        {
            return new FVector3(s / v.x, s / v.y, s / v.z);
        }

        /// <summary>
        /// Division operator of a vector by a scalar (as <c>int</c>).
        /// </summary>
        /// <remarks>
        /// Slightly faster than <c>operator/(FScalar s, FVector2 v)</c>.
        /// </remarks>
        public static FVector3 operator/(int s, FVector3 v)
        {
            return new FVector3(s / v.x, s / v.y, s / v.z);
        }

        /// <summary>
        /// Equality operator.
        /// </summary>
        public static bool operator==(FVector3 a, FVector3 b)
        {
            return a.x == b.x && a.y == b.y && a.z == b.z;
        }

        /// <summary>
        /// Inequality operator.
        /// </summary>
        public static bool operator!=(FVector3 a, FVector3 b)
        {
            return a.x != b.x || a.y != b.y || a.z != b.z;
        }

        #endregion

        #region Static methods

        /// <summary>
        /// 判断两向量是否近似想等, 夹角小于阀值
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        public static bool IsApproximatelyAngleZero(FVector3 a, FVector3 b)
        {
            var angle = Angle(a, b);
            return angle <= FScalar.epsilon;
        }

        /// <summary>
        /// Angle in turns between two (non-normalized) vectors.
        /// </summary>
        /// <returns>Angle in turns in [0, 0.5], i.e. 0 to 180 degrees.
        public static FScalar Angle(FVector3 from, FVector3 to)
        {
            return FScalar.Acos(FScalar.Clamp(Dot(from.normalized, to.normalized), -FScalar.one, FScalar.one));
        }

        /// <summary>
        /// Angle in turns between two normalized vectors.
        /// </summary>
        /// <returns>Angle in turns in [0, 0.5], i.e. 0 to 180 degrees.
        /// <remarks>This is faster than Angle(), which needs normalizations internally.</remarks>
        public static FScalar AngleNormalized(FVector3 from, FVector3 to)
        {
            return FScalar.Acos(FScalar.Clamp(Dot(from, to), -FScalar.one, FScalar.one));
        }

        /// <summary>
        /// Returns a vector with its <c>magnitude</c> clamped to <c>maxLength</c>.
        /// </summary>
        /// <param name="v">Vector to be clamped.</param>
        /// <param name="maxLength">Maximum length.</param>
        /// <returns>The clamped magnitude.</returns>
        public static FVector3 ClampMagnitude(FVector3 v, FScalar maxLength)
        {
            FScalar m = v.magnitude;
            if (m > maxLength)
                return v * (FScalar.one /  m);
            else
                return v;
        }

        /// <summary>
        /// Compute the distance between two points.
        /// </summary>
        /// <param name="a">The first point.</param>
        /// <param name="b">The second point.</param>
        public static FScalar Distance(FVector3 a, FVector3 b)
        {
            return (a - b).magnitude;
        }

        /// <summary>
        /// Compute the squared distance between two points.
        /// </summary>
        /// <returns>The squared distance.</returns>
        /// <param name="a">The first point.</param>
        /// <param name="b">The second point.</param>
        /// <remarks>
        /// This is faster than <c>Distance()</c> because of removal of a <c>Sqrt()</c> operation.
        /// </remarks>
        public static FScalar SqrDistance(FVector3 a, FVector3 b)
        {
            return (a - b).sqrMagnitude;
        }

        /// <summary>
        /// Compute the squared distance between two points and returns in raw format.
        /// </summary>
        /// <returns>The squared distance in 44.20 raw format.</returns>
        /// <param name="a">The alpha component.</param>
        /// <param name="b">The blue component.</param>
        public static long SqrDistanceRaw(FVector3 a, FVector3 b)
        {
            return (a - b).sqrMagnitudeRaw;
        }

        /// <summary>
        /// Determines if the distance between two points is less than the specified distance.
        /// </summary>
        /// <returns>
        /// <c>true</c>If the distnace between two points is less than the specified <c>distance</c>;
        /// otherwise, <c>false</c>.
        /// </returns>
        /// <param name="a">The alpha component.</param>
        /// <param name="b">The blue component.</param>
        /// <param name="distance">The specified distance.</param>
        public static bool IsDistanceLess(FVector3 a, FVector3 b, FScalar FDistance)
        {
            return (a - b).sqrMagnitudeRaw < FScalar.MultiplyRaw(FDistance, FDistance);
        }

        /// <summary>
        /// Determines if the distance between two points is less than or equal to the specified distance.
        /// </summary>
        /// <returns>
        /// <c>true</c>If the distnace between two points is less than or equal to the specified <c>distance</c>;
        /// otherwise, <c>false</c>.
        /// </returns>
        /// <param name="a">The alpha component.</param>
        /// <param name="b">The blue component.</param>
        /// <param name="distance">The specified distance.</param>
        public static bool IsDistanceLessEqual(FVector3 a, FVector3 b, FScalar FDistance)
        {
            return (a - b).sqrMagnitudeRaw <= FScalar.MultiplyRaw(FDistance, FDistance);
        }


        /// <summary>
        /// Dot product of two vectors.
        /// </summary>
        /// <param name="a">The first vector.</param>
        /// <param name="b">The second vector.</param>
        /// <remarks>
        /// Dot product is commutative: <c></c>FVector2.Dot(a, b) == FVector2.Dot(b, a)</c>
        /// </remarks>
        public static FScalar Dot(FVector3 a, FVector3 b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        /// <summary>
        /// Dot product of two vectors in raw format.
        /// </summary>
        /// <param name="a">The first vector.</param>
        /// <param name="b">The second vector.</param>
        /// <returns>Dot product in 44.20 raw format</returns>
        /// <remarks>
        /// Dot product is commutative: <c></c>FVector2.Dot(a, b) == FVector2.Dot(b, a)</c>
        /// </remarks>
        public static long DotRaw(FVector3 a, FVector3 b)
        {
            return (long)a.x.rawValue * (long)b.x.rawValue + (long)a.y.rawValue * (long)b.y.rawValue + (long)a.z.rawValue * (long)b.z.rawValue;
        }

        /// <summary>
        /// Linearly interpolates between vectors a and b by t.
        /// </summary>
        /// <param name="a">The first vector.</param>
        /// <param name="b">The second vector.</param>
        /// <param name="t">The interpolation parameter, which will be clamped to [0, 1].</param>
        public static FVector3 Lerp(FVector3 a, FVector3 b, FScalar t)
        {
            t = FScalar.Saturate(t);
            return (FScalar.one - t) * a + t * b;
        }

        /// <summary>
        /// Linearly interpolates between vectors a and b by t, without clamping.
        /// </summary>
        /// <param name="a">The first vector.</param>
        /// <param name="b">The second vector.</param>
        /// <param name="t">The interpolation parameter.</param>
        public static FVector3 LerpUnclamped(FVector3 a, FVector3 b, FScalar t)
        {
            return (FScalar.one - t) * a + t * b;
        }

        /// <summary>
        /// xz 平面旋转固定角度
        /// </summary>
        /// <param name="a">原向量</param>
        /// <param name="t">[0,1]角度值</param>
        /// <param name="fixVector">修正向量，朝该向量方向顺时针或逆时针修正</param>
        /// <returns></returns>
        public static FVector3 RotateXZAngle(FVector3 a, FScalar t, FVector3 fixVector = default(FVector3))
        {
            if (fixVector != default(FVector3))
            {
                FScalar cross = a.x * fixVector.z - a.z * fixVector.x;
                if (cross > 0)
                    t = 1 - t;
            }
            FScalar cost = FScalar.Cos(t);
            FScalar sint = FScalar.Sin(t);
            return new FVector3(cost * a.x + sint * a.z, 0, cost * a.z - sint * a.x);
        }

        /// <summary>
        ///  Slerp (normalized).
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="t"></param>
        /// <returns></returns>
        public static FVector3 SLerpNormalized(FVector3 a, FVector3 b, FScalar t)
        {
            FScalar doot = Dot(a, b);
            doot = FScalar.Clamp(doot, -1, 1);
            var theta = FScalar.Acos(doot) * t;
            var relativeVec = b - a * doot;
            relativeVec.Normalize();
            return a * FScalar.Cos(theta) + relativeVec * FScalar.Sin(theta);
        }

        /// <summary>
        /// Component-wise absolute values.
        /// </summary>
        public static FVector3 Abs(FVector3 v)
        {
            return new FVector3(FScalar.Abs(v.x), FScalar.Abs(v.y), FScalar.Abs(v.z));
        }

        /// <summary>
        /// Component-wise maximum between two vectors.
        /// </summary>
        public static FVector3 Max(FVector3 a, FVector3 b)
        {
            return new FVector3(FScalar.Max(a.x, b.x), FScalar.Max(a.y, b.y), FScalar.Max(a.z, b.z));
        }

        /// <summary>
        /// Component-wise minimum between two vectors.
        /// </summary>
        public static FVector3 Min(FVector3 a, FVector3 b)
        {
            return new FVector3(FScalar.Min(a.x, b.x), FScalar.Min(a.y, b.y), FScalar.Min(a.z, b.z));
        }

        /// <summary>
        /// Moves a point current towards target.
        /// </summary>
        /// <returns>The towards.</returns>
        /// <param name="current">Current position.</param>
        /// <param name="target">Target position.</param>
        /// <param name="maxDistanceDelta">Maximum distance that the point can move.</param>
        public static FVector3 MoveTowards(FVector3 current, FVector3 target, FScalar maxDistanceDelta)
        {
            FVector3 u = target - current;
            long ll = u.sqrMagnitudeRaw;
            long dd = FScalar.MultiplyRaw(maxDistanceDelta, maxDistanceDelta);
            if (ll <= dd)
                return target;
            else
                return current + u * (maxDistanceDelta / FScalar.SqrtRaw(ll));
        }

        /// <summary>
        /// Component-wise multiplication (Hadamard product).
        /// </summary>
        /// <param name="a">The first vector.</param>
        /// <param name="b">The second vector.</param>
        public static FVector3 Scale(FVector3 a, FVector3 b)
        {
            return new FVector3(a.x * b.x, a.y * b.y, a.z * b.z);
        }

        /// <summary>
        /// Component-wise multiplication (Hadamard product).
        /// </summary>
        /// <param name="scale">The scaling vector.</param>
        public void Scale(FVector3 scale)
        {
            x *= scale.x;
            y *= scale.y;
            z *= scale.z;
        }
            
        /// <summary>
        /// Cross product of <c>a</c> and <c>b</c>.
        /// </summary>
        /// <param name="a">The first vector.</param>
        /// <param name="b">The second vector.</param>
        public static FVector3 Cross(FVector3 a, FVector3 b)
        {
            return new FVector3(
                    a.y * b.z - a.z * b.y,
                    a.z * b.x - a.x * b.z,
                    a.x * b.y - a.y * b.x);
        }

        /// <summary>
        /// Project a vector on a (non-normalized) direction.
        /// </summary>
        /// <param name="v">Vector to be projected,</param>
        /// <param name="direction">The direction of projection.</param>
        public static FVector3 Project(FVector3 v, FVector3 direction)
        {
            FVector3 n = direction.normalized;
            return FVector3.Dot(v, n) * n;
        }

        /// <summary>
        /// Project a vector on a normalized direction.
        /// </summary>
        /// <param name="v">Vector to be projected,</param>
        /// <param name="direction">The direction of projection.</param>
        public static FVector3 ProjectNormalized(FVector3 v, FVector3 direction)
        {
            return FVector3.Dot(v, direction) * direction;
        }

        /// <summary>
        /// Reflects a vector off the vector defined by a normal.
        /// </summary>
        /// <param name="inDirection">Direction to be reflected.</param>
        /// <param name="inNormal">Normal vector of the reflection plane (must be normalized).</param>
        public static FVector3 Reflect(FVector3 inDirection, FVector3 inNormal)
        {
            return (-FScalar.two * FVector3.Dot(inNormal, inDirection)) * inNormal + inDirection;
        }

        /// <summary>
        /// Projects a vector on a plane.
        /// </summary>
        /// <param name="inDirection">Direction to be Projected.</param>
        /// <param name="inNormal">Normal vector of the projection plane (must be normalized).</param>
        public static FVector3 ProjectOnPlane(FVector3 vec, FVector3 onNormal)
        {
            return vec - onNormal * FVector3.Dot(vec, onNormal);
        }
        
        /// <summary>
        /// 向量沿Y轴旋转
        /// </summary>
        /// <param name=""></param>
        /// <param name="deg">0-360</param>
        public static FVector3 PlaneRotate(FVector3 vec, FScalar deg)
        {
            //1.normal
            FScalar len = vec.magnitude;
            var ret = vec;
            if (len > 0)
            {
                vec /= len;
            }

            //2.cos, sin
            FScalar cosx = 0;
            FScalar sinx = 0;
            FScalar.SinCos(deg / 360, out sinx, out cosx);

            //3.rotate & restore
            ret.x = (cosx * vec.x + sinx * vec.z) * len;
            ret.z = (cosx * vec.z - sinx * vec.x) * len;
            return ret;
        }
        /// <summary>
        /// 向量沿Z轴旋转
        /// </summary>
        /// <param name=""></param>
        /// <param name="deg">0-360</param>
        public static FVector3 PlaneRotateByZ(FVector3 vec, FScalar deg)
        {
            //1.normal
            FScalar len = vec.magnitude;
            var ret = vec;
            if (len > 0)
            {
                vec /= len;
            }

            //2.cos, sin
            FScalar cosx = 0;
            FScalar sinx = 0;
            FScalar.SinCos(deg / 360, out sinx, out cosx);

            //3.rotate & restore
            ret.x = (cosx * vec.x - sinx * vec.y) * len;
            ret.y = (cosx * vec.y + sinx * vec.x) * len;
            return ret;
        }
        /// <summary>
        /// 向量沿X轴旋转
        /// </summary>
        /// <param name=""></param>
        /// <param name="deg">0-360</param>
        public static FVector3 PlaneRotateByX(FVector3 vec, FScalar deg)
        {
            //1.normal
            FScalar len = vec.magnitude;
            var ret = vec;
            if (len > 0)
            {
                vec /= len;
            }

            //2.cos, sin
            FScalar cosx = 0;
            FScalar sinx = 0;
            FScalar.SinCos(deg / 360, out sinx, out cosx);

            //3.rotate & restore
            ret.y = (cosx * vec.y - sinx * vec.z) * len;
            ret.z = (cosx * vec.z + sinx * vec.y) * len;
            return ret;
        }
        #endregion

        #region IEquatable<FVector3>

        public bool Equals(FVector3 rhs)
        {
            return x == rhs.x && y == rhs.y && z == rhs.z;
        }

        #endregion

        #region Object
                
        public override bool Equals(object obj)
        {
            return obj is FVector3 && ((FVector3)obj) == this;
        }
        
        public override int GetHashCode()
        {
            int hash = 17;
            hash = hash * 31 + x.GetHashCode();
            hash = hash * 31 + y.GetHashCode();
            hash = hash * 31 + z.GetHashCode();
            return hash;
        }

        public override string ToString()
        {
            return x + ", " + y + ", " + z;
        }

        #endregion
    }
}
