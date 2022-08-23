using System;
using System.Runtime.InteropServices;
namespace Morefun.LockStep
{
    [Serializable, StructLayout(LayoutKind.Sequential)]
    public struct FVector2 : IEquatable<FVector2>
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
        #endregion

        #region Constants

        /// <summary>
        /// The zero vector.
        /// </summary>
        public static readonly FVector2 zero = new FVector2(FScalar.zero, FScalar.zero);

        /// <summary>
        /// The vector of (1, 1).
        /// </summary>
        public static readonly FVector2 one = new FVector2(FScalar.one, FScalar.one);

        /// <summary>
        /// The vector of minimum components.
        /// </summary>
        public static readonly FVector2 min = new FVector2(FScalar.minValue, FScalar.minValue);

        /// <summary>
        /// The vector of maximum components.
        /// </summary>
        public static readonly FVector2 max = new FVector2(FScalar.maxValue, FScalar.maxValue);

        /// <summary>
        /// The back direction (0, -1).
        /// </summary>
        public static readonly FVector2 back = new FVector2(FScalar.zero, -FScalar.one);

        /// <summary>
        /// The left direction (-1, 0).
        /// </summary>
        public static readonly FVector2 left    = new FVector2(-FScalar.one,   FScalar.zero);

        /// <summary>
        /// The right direction (1, 0).
        /// </summary>
        public static readonly FVector2 right   = new FVector2( FScalar.one,   FScalar.zero);

        /// <summary>
        /// The forward direction (0, 1).
        /// </summary>
        public static readonly FVector2 forward = new FVector2( FScalar.zero,  FScalar.one );

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
        /// When this is a zero vector, it returns a zero vector, without throwing division-by-zero exception.
        /// </description>
        public FVector2 normalized
        {
            get
            {
                long d = sqrMagnitudeRaw;
                if (d == 0)
                    return FVector2.zero;
                else
                    return this * FScalar.InvSqrtRaw(d);
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
                if (index == 0)
                    return x;
                else if (index == 1)
                    return y;
                else
                    throw new IndexOutOfRangeException();
            }
            set
            {
                if (index == 0)
                    x = value;
                else if (index == 1)
                    y = value;
                else
                    throw new IndexOutOfRangeException();
            }
        }

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a FVector2.
        /// </summary>
        /// <param name="x">The x component.</param>
        /// <param name="y">The y component.</param>
        public FVector2(FScalar x, FScalar y)
        {
            this.x = x;
            this.y = y;
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
        public static FVector2 operator+(FVector2 v)
        {
            return v;
        }

        /// <summary>
        /// Negation operator.
        /// </summary>
        public static FVector2 operator-(FVector2 v)
        {
            return new FVector2(-v.x, -v.y);
        }

        /// <summary>
        /// Addition operator.
        /// </summary>
        public static FVector2 operator+(FVector2 a, FVector2 b)
        {
            return new FVector2(a.x + b.x, a.y + b.y);
        }

        /// <summary>
        /// Subtraction operator.
        /// </summary>
        public static FVector2 operator-(FVector2 a, FVector2 b)
        {
            return new FVector2(a.x - b.x, a.y - b.y);
        }

        /// <summary>
        /// Multiplication operator of a vector and a scalar.
        /// </summary>
        public static FVector2 operator*(FVector2 v, FScalar s)
        {
            return new FVector2(v.x * s, v.y * s);
        }

        /// <summary>
        /// Multiplication operator of a vector and a scalar (as int).
        /// </summary>
        /// <remarks>
        /// Slightly faster than <c>operator*(FVector2 v, FScalar s)</c>.
        /// </remarks>
        public static FVector2 operator*(FVector2 v, int s)
        {
            return new FVector2(v.x * s, v.y * s);
        }

        /// <summary>
        /// Multiplication operator of a scalar and a vector.
        /// </summary>
        public static FVector2 operator*(FScalar s, FVector2 v)
        {
            return new FVector2(s * v.x, s * v.y);
        }

        /// <summary>
        /// Multiplication operator of a scalar (as <c>int</c>) and a vector.
        /// </summary>
        /// <remarks>
        /// Slightly faster than <c>operator*(FScalar s, FVector2 v)</c>.
        /// </remarks>
        public static FVector2 operator*(int s, FVector2 v)
        {
            return new FVector2(s * v.x, s * v.y);
        }

        /// <summary>
        /// Division operator of a vector by a scalar.
        /// </summary>
        public static FVector2 operator/(FVector2 v, FScalar s)
        {
            return new FVector2(v.x / s, v.y / s);
        }

        /// <summary>
        /// Division operator of a vector by a scalar (as <c>int</c>).
        /// </summary>
        /// <remarks>
        /// Slightly faster than <c>operator/(FVector2 v, FScalar s)</c>.
        /// </remarks>
        public static FVector2 operator/(FVector2 v, int s)
        {
            return new FVector2(v.x / s, v.y / s);
        }

        /// <summary>
        /// Division operator of a scalar by a vector.
        /// </summary>
        public static FVector2 operator/(FScalar s, FVector2 v)
        {
            return new FVector2(s / v.x, s / v.y);
        }

        /// <summary>
        /// Division operator of a vector by a scalar (as <c>int</c>).
        /// </summary>
        /// <remarks>
        /// Slightly faster than <c>operator/(FScalar s, FVector2 v)</c>.
        /// </remarks>
        public static FVector2 operator/(int s, FVector2 v)
        {
            return new FVector2(s / v.x, s / v.y);
        }

        /// <summary>
        /// Equality operator.
        /// </summary>
        public static bool operator==(FVector2 a, FVector2 b)
        {
            return a.x == b.x && a.y == b.y;
        }

        /// <summary>
        /// Inequality operator.
        /// </summary>
        public static bool operator!=(FVector2 a, FVector2 b)
        {
            return a.x != b.x || a.y != b.y;
        }

        #endregion

        #region Static functions

        /// <summary>
        /// Angle in turns between two (non-normalized) vectors.
        /// </summary>
        /// <returns>Angle in turns in [0, 0.5], i.e. 0 to 180 degrees.
        public static FScalar Angle(FVector2 from, FVector2 to)
        {
            return FScalar.Acos(FScalar.Clamp(Dot(from.normalized, to.normalized), -FScalar.one, FScalar.one));
        }

        /// <summary>
        /// Angle in turns between two normalized vectors.
        /// </summary>
        /// <returns>Angle in turns in [0, 0.5], i.e. 0 to 180 degrees.
        /// <remarks>This is faster than Angle(), which needs normalizations internally.</remarks>
        public static FScalar AngleNormalized(FVector2 from, FVector2 to)
        {
            return FScalar.Acos(FScalar.Clamp(Dot(from, to), -FScalar.one, FScalar.one));
        }
        /// <summary>
        /// AngleRelate in turns between two (non-normalized) vectors.
        /// </summary>
        /// <returns>Angle in turns in [-0.5, 0.5], i.e. -180 to 180 degrees.
        public static FScalar AngleRelate(FVector2 from, FVector2 to)
        {
            var angleFrom = FScalar.Atan2(from.y, from.x);
            var angleTo = FScalar.Atan2(to.y, to.x);
            var angle = angleTo - angleFrom;
            var anglefinal = angle;
            //fix to -180 to 180 degrees.
            if (angle > FScalar.half)
                anglefinal -= FScalar.one;
            else if (angle < -FScalar.half)
                anglefinal += FScalar.one;
//            LockstepDebug.LogWarning("from " + angleFrom + " to" + angleTo + " ang= " + angle + " fin " + anglefinal) ;
            return anglefinal;
        }

        /// <summary>
        /// Returns a vector with its <c>magnitude</c> clamped to <c>maxLength</c>.
        /// </summary>
        /// <param name="v">Vector to be clamped.</param>
        /// <param name="maxLength">Maximum length.</param>
        /// <returns>The clamped magnitude.</returns>
        public static FVector2 ClampMagnitude(FVector2 v, FScalar maxLength)
        {
            FScalar m = v.magnitude;
            if (m > maxLength)
                return v * (maxLength /  m);
            else
                return v;
        }

        /// <summary>
        /// Compute the distance between two points.
        /// </summary>
        /// <param name="a">The first point.</param>
        /// <param name="b">The second point.</param>
        public static FScalar Distance(FVector2 a, FVector2 b)
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
        public static FScalar SqrDistance(FVector2 a, FVector2 b)
        {
            return (a - b).sqrMagnitude;
        }

        /// <summary>
        /// Compute the squared distance between two points and returns in raw format.
        /// </summary>
        /// <returns>The squared distance in 44.20 raw format.</returns>
        /// <param name="a">The alpha component.</param>
        /// <param name="b">The blue component.</param>
        public static long SqrDistanceRaw( FVector2 a, FVector2 b)
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
        public static bool IsDistanceLess(FVector2 a, FVector2 b, FScalar distance)
        {
            return (a - b).sqrMagnitudeRaw < FScalar.MultiplyRaw(distance, distance);
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
        public static bool IsDistanceLessEqual(FVector2 a, FVector2 b, FScalar FDistance)
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
        public static FScalar Dot(FVector2 a, FVector2 b)
        {
            return a.x * b.x + a.y * b.y;
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
        public static long DotRaw(FVector2 a, FVector2 b)
        {
            return (long)a.x.rawValue * (long)b.x.rawValue + (long)a.y.rawValue * (long)b.y.rawValue;
        }

        /// <summary>
        /// Linearly interpolates between vectors a and b by t.
        /// </summary>
        /// <param name="a">The first vector.</param>
        /// <param name="b">The second vector.</param>
        /// <param name="t">The interpolation parameter, which will be clamped to [0, 1].</param>
        public static FVector2 Lerp(FVector2 a, FVector2 b, FScalar t)
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
        public static FVector2 LerpUnclamped(FVector2 a, FVector2 b, FScalar t)
        {
            return (FScalar.one - t) * a + t * b;
        }

        /// <summary>
        /// Component-wise absolute values.
        /// </summary>
        public static FVector2 Abs(FVector2 v)
        {
            return new FVector2(FScalar.Abs(v.x), FScalar.Abs(v.y));
        }

        /// <summary>
        /// Component-wise maximum between two vectors.
        /// </summary>
        public static FVector2 Max(FVector2 a, FVector2 b)
        {
            return new FVector2(FScalar.Max(a.x, b.x), FScalar.Max(a.y, b.y));
        }

        /// <summary>
        /// Component-wise minimum between two vectors.
        /// </summary>
        public static FVector2 Min(FVector2 a, FVector2 b)
        {
            return new FVector2(FScalar.Min(a.x, b.x), FScalar.Min(a.y, b.y));
        }

        /// <summary>
        /// Moves a point current towards target.
        /// </summary>
        /// <returns>The towards.</returns>
        /// <param name="current">Current position.</param>
        /// <param name="target">Target position.</param>
        /// <param name="maxDistanceDelta">Maximum distance that the point can move.</param>
        public static FVector2 MoveTowards(FVector2 current, FVector2 target, FScalar maxDistanceDelta)
        {
            FVector2 u = target - current;
            return u * FScalar.Min(u.magnitude, maxDistanceDelta);
        }

        /// <summary>
        /// Component-wise multiplication (Hadamard product).
        /// </summary>
        /// <param name="a">The first vector.</param>
        /// <param name="b">The second vector.</param>
        public static FVector2 Scale(FVector2 a, FVector2 b)
        {
            return new FVector2(a.x * b.x, a.y * b.y);
        }

        /// <summary>
        /// Component-wise multiplication (Hadamard product).
        /// </summary>
        /// <param name="scale">The scaling vector.</param>
        public void Scale(FVector2 scale)
        {
            x *= scale.x;
            y *= scale.y;
        }

        /// <summary>
        /// Gradually changes a vector towards a desired goal over time.
        /// </summary>
        /// <param name="current">The current position.</param>
        /// <param name="target">The position we are trying to reach.</param>
        /// <param name="currentVelocity">The current velocity, this value is modified by the function every time you call it.</param>
        /// <param name="smoothTime">Approximately the time it will take to reach the target. A smaller value will reach the target faster.</param>
        /// <param name="maxSpeed">Optionally allows you to clamp the maximum speed.</param>
        /// <param name="deltaTime">The time since the last call to this function. By default Time.deltaTime.</param>
        public static FVector2 SmoothDamp(FVector2 current, FVector2 target, ref FVector2 currentVelocity, FScalar smoothTime, FScalar maxSpeed, FScalar deltaTime)
        {
            smoothTime = FScalar.Max(FScalar.epsilon, smoothTime);
            FScalar num = FScalar.one / smoothTime;
            FScalar num2 = num * deltaTime;
            // 0.48 \approx 492 / 1024
            // 0.235 \approx 241 / 1024
            FScalar d = FScalar.one / (FScalar.one + num2 + FScalar.FromRaw(492) * num2 * num2 + FScalar.FromRaw(241) * num2 * num2 * num2);
            FVector2 vector = current - target;
            FVector2 vector2 = target;
            FScalar maxLength = maxSpeed * smoothTime;
            vector = FVector2.ClampMagnitude(vector, maxLength);
            target = current - vector;
            FVector2 vector3 = (currentVelocity + num * vector) * deltaTime;
            currentVelocity = (currentVelocity - num * vector3) * d;
            FVector2 vector4 = target + (vector + vector3) * d;
            if (FVector2.Dot(vector2 - current, vector4 - vector2) > FScalar.zero)
            {
                vector4 = vector2;
                currentVelocity = (vector4 - vector2) / deltaTime;
            }
            return vector4;
        }

        public static FVector2 SmoothDamp(FVector2 current, FVector2 target, ref FVector2 currentVelocity, FScalar smoothTime)
        {
            //TODO deltaTime
            //return FVector2.SmoothDamp(current, target, ref currentVelocity, smoothTime, FScalar.maxValue, LTime.deltaTime);
            return new FVector2(0,0);
        }

        public static FVector2 SmoothDamp(FVector2 current, FVector2 target, ref FVector2 currentVelocity, FScalar smoothTime, FScalar maxSpeed)
        {
            //TODO deltaTime
            //return FVector2.SmoothDamp(current, target, ref currentVelocity, smoothTime, maxSpeed, LTime.deltaTime);
            return new FVector2(0,0);
        }

        /// <summary>
        /// Cross product of <c>a</c> and <c>b</c>.
        /// </summary>
        /// <param name="a">The first vector with z = 0.</param>
        /// <param name="b">The second vector with z = 0.</param>
        /// <returns>Z-component of the cross-product.</returns>
        public static FScalar Cross(FVector2 a, FVector2 b)
        {
            return a.x * b.y - b.x * a.y;
        }

        /// <summary>
        /// Cross product of <c>a</c> and <c>b</c> in raw format.
        /// </summary>
        /// <param name="a">The first vector with z = 0.</param>
        /// <param name="b">The second vector with z = 0.</param>
        /// <returns>Z-component of the cross-product in 44.20 raw format.</returns>
        public static long CrossRaw(FVector2 a, FVector2 b)
        {
            return FScalar.MultiplyRaw(a.x, b.y) - FScalar.MultiplyRaw(b.x, a.y);
        }

        /// <summary>
        /// Project a vector on a (non-normalized) direction.
        /// </summary>
        /// <param name="v">Vector to be projected,</param>
        /// <param name="direction">The direction of projection.</param>
        public static FVector2 Project(FVector2 v, FVector2 direction)
        {
            FVector2 n = direction.normalized;
            return FVector2.Dot(v, n) * n;
        }

        /// <summary>
        /// Project a vector on a normalized direction.
        /// </summary>
        /// <param name="v">Vector to be projected,</param>
        /// <param name="direction">The direction of projection.</param>
        public static FVector2 ProjectNormalized(FVector2 v, FVector2 direction)
        {
            return FVector2.Dot(v, direction) * direction;
        }

        /// <summary>
        /// Rotates a vector with a rotation unit vector.
        /// </summary>
        /// <returns>The vector.</returns>
        /// <param name="v">Vector to be rotated.</param>
        /// <param name="rotation">Rotation unit vector (cos angle, sin angle).</param>
        public static FVector2 RotateVector(FVector2 v, FVector2 rotation)
        {
            return new FVector2(
                v.x * rotation.x - v.y * rotation.y,
                v.x * rotation.y + v.y * rotation.x);
        }

        /// <summary>
        /// Rotates a vector with an angle. 
        /// </summary>
        /// <returns>The vector.</returns>
        /// <param name="v">Vector to be rotated.</param>
        /// <param name="angle">Angle of rotation in turns.</param>
        public static FVector2 RotateVector(FVector2 v, FScalar angle)
        {
            FVector2 rotation;
            FScalar.SinCos(angle, out rotation.y, out rotation.x);
            return RotateVector(v, rotation);
        }

        // <summary>
        /// Rotates a (non-normalized) vector current towards target.
        /// </summary>
        /// <param name="current">current direction.</param>
        /// <param name="target">target direction.</param>
        /// <param name="deltaAngle">Delta angle in terms of turns.</param>
        public static FVector2 RotateTowards(FVector2 current , FVector2 target, FScalar deltaAngle)
        {
            if (deltaAngle >= FVector2.Angle(current, target))
                return target;
            else
                return FVector2.RotateVector(current, FVector2.CrossRaw(current, target) > 0 ? deltaAngle : -deltaAngle);
        }

        // <summary>
        /// Rotates a normalized vector current towards normalized target.
        /// </summary>
        /// <param name="current">Normalized current direction.</param>
        /// <param name="target">Normalized target direction.</param>
        /// <param name="deltaAngle">Delta angle in terms of turns.</param>
        public static FVector2 RotateTowardsNormalized(FVector2 current , FVector2 target, FScalar deltaAngle)
        {
            if (deltaAngle >= FVector2.AngleNormalized(current, target))
                return target;
            else
                return FVector2.RotateVector(current, FVector2.CrossRaw(current, target) > 0 ? deltaAngle : -deltaAngle);
        }

        /// <summary>
        /// Reflects a vector off the vector defined by a normal.
        /// </summary>
        /// <param name="inDirection">Direction to be reflected.</param>
        /// <param name="inNormal">Normal vector of the reflection plane (must be normalized).</param>
        public static FVector2 Reflect(FVector2 inDirection, FVector2 inNormal)
        {
            return (-FScalar.two * FVector2.Dot(inNormal, inDirection)) * inNormal + inDirection;
        }

        #endregion

        #region IEquatable<FVector2>
        
        public bool Equals(FVector2 rhs)
        {
            return x == rhs.x && y == rhs.y;
        }

        #endregion

        #region Object
        
        public override bool Equals(object obj)
        {
            return obj is FVector2 && ((FVector2)obj) == this;
        }
        
        public override int GetHashCode()
        {
            int hash = 17;
            hash = hash * 31 + x.GetHashCode();
            hash = hash * 31 + y.GetHashCode();
            return hash;
        }

        public override string ToString()
        {
            return x + ", " + y;
        }

        #endregion
    }
}
