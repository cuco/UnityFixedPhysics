using System;
using System.Runtime.CompilerServices;
using Unity.IL2CPP.CompilerServices;
using static Unity.Mathematics.math;
using static Unity.Mathematics.FixedPoint.fpmath;

namespace Unity.Mathematics.FixedPoint
{
    /// <summary>
    /// A fpQuaternion type for representing rotations.
    /// </summary>
    //[Il2CppEagerStaticClassConstruction]
    [Serializable]
    public partial struct fpquaternion : IEquatable<fpquaternion>, IFormattable
    {
        /// <summary>
        /// The fpQuaternion component values.
        /// </summary>
        public fp4 value;

        /// <summary>A fpQuaternion representing the identity transform.</summary>
        public static readonly fpquaternion identity = new fpquaternion(fp.zero, fp.zero, fp.zero, fp.one);

        /// <summary>Constructs a fpQuaternion from four fp values.</summary>
        /// <param name="x">The fpQuaternion x component.</param>
        /// <param name="y">The fpQuaternion y component.</param>
        /// <param name="z">The fpQuaternion z component.</param>
        /// <param name="w">The fpQuaternion w component.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fpquaternion(fp x, fp y, fp z, fp w) { value.x = x; value.y = y; value.z = z; value.w = w; }

        /// <summary>Constructs a fpQuaternion from fp4 vector.</summary>
        /// <param name="value">The fpQuaternion xyzw component values.</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public fpquaternion(fp4 value) { this.value = value; }

        /// <summary>Implicitly converts a fp4 vector to a fpQuaternion.</summary>
        /// <param name="v">The fpQuaternion xyzw component values.</param>
        /// <returns>The fpQuaternion constructed from a fp4 vector.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator fpquaternion(fp4 v) { return new fpquaternion(v); }

        /// <summary>Constructs a unit fpQuaternion from a fp3x3 rotation matrix. The matrix must be orthonormal.</summary>
        /// <param name="m">The fp3x3 orthonormal rotation matrix.</param>
        public fpquaternion(fp3x3 m)
        {
            fp3 u = m.c0;
            fp3 v = m.c1;
            fp3 w = m.c2;

            uint u_sign = (asuint(u.x) & 0x80000000);
            fp t = v.y + (fp)(asuint(w.z) ^ u_sign);
            uint4 u_mask = uint4((int)u_sign >> 31);
            uint4 t_mask = uint4((int)(t) >> 31);

            fp tr = fp.one + abs(u.x);

            uint4 sign_flips = uint4(0x00000000, 0x80000000, 0x80000000, 0x80000000) ^ (u_mask & uint4(0x00000000, 0x80000000, 0x00000000, 0x80000000)) ^ (t_mask & uint4(0x80000000, 0x80000000, 0x80000000, 0x00000000));

            value = new fp4(tr, u.y, w.x, v.z) + fp4(asuint(new fp4(t, v.x, u.z, w.y)) ^ sign_flips);   // +---, +++-, ++-+, +-++

            value = fp4((asuint(value) & ~u_mask) | (asuint(value.zwxy) & u_mask));
            value = fp4((asuint(value.wzyx) & ~t_mask) | (asuint(value) & t_mask));
            value = normalize(value);
        }

        /// <summary>Constructs a unit fpQuaternion from an orthonormal fp4x4 matrix.</summary>
        /// <param name="m">The fp4x4 orthonormal rotation matrix.</param>
        public fpquaternion(fp4x4 m)
        {
            fp4 u = m.c0;
            fp4 v = m.c1;
            fp4 w = m.c2;

            uint u_sign = (asuint(u.x) & 0x80000000);
            fp t = v.y + (fp)(asuint(w.z) ^ u_sign);
            uint4 u_mask = uint4((int)u_sign >> 31);
            uint4 t_mask = uint4((int)(t) >> 31);

            fp tr = fp.one + abs(u.x);

            uint4 sign_flips = uint4(0x00000000, 0x80000000, 0x80000000, 0x80000000) ^ (u_mask & uint4(0x00000000, 0x80000000, 0x00000000, 0x80000000)) ^ (t_mask & uint4(0x80000000, 0x80000000, 0x80000000, 0x00000000));

            value = new fp4(tr, u.y, w.x, v.z) + fp4(asuint(new fp4(t, v.x, u.z, w.y)) ^ sign_flips);   // +---, +++-, ++-+, +-++

            value = fp4((asuint(value) & ~u_mask) | (asuint(value.zwxy) & u_mask));
            value = fp4((asuint(value.wzyx) & ~t_mask) | (asuint(value) & t_mask));

            value = normalize(value);
        }

        /// <summary>
        /// Returns a fpQuaternion representing a rotation around a unit axis by an angle in radians.
        /// The rotation direction is clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="axis">The axis of rotation.</param>
        /// <param name="angle">The angle of rotation in radians.</param>
        /// <returns>The fpQuaternion representing a rotation around an axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion AxisAngle(fp3 axis, fp angle)
        {
            fp sina, cosa;
            sincos(fp.half * angle, out sina, out cosa);
            return fpQuaternion(fp4(axis * sina, cosa));
        }

        /// <summary>
        /// Returns a fpQuaternion constructed by first performing a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fpQuaternion representing the Euler angle rotation in x-y-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion EulerXYZ(fp3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateY(xyz.y), rotateX(xyz.x)));
            fp3 s, c;
            sincos(fp.half * xyz, out s, out c);
            return fpQuaternion(
                // s.x * c.y * c.z - s.y * s.z * c.x,
                // s.y * c.x * c.z + s.x * s.z * c.y,
                // s.z * c.x * c.y - s.x * s.y * c.z,
                // c.x * c.y * c.z + s.y * s.z * s.x
                fp4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * fp4(c.xyz, s.x) * fp4(fp.minusOne, fp.one, fp.minusOne, fp.one)
                );
        }

        /// <summary>
        /// Returns a fpQuaternion constructed by first performing a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fpQuaternion representing the Euler angle rotation in x-z-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion EulerXZY(fp3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateZ(xyz.z), rotateX(xyz.x)));
            fp3 s, c;
            sincos(fp.half * xyz, out s, out c);
            return fpQuaternion(
                // s.x * c.y * c.z + s.y * s.z * c.x,
                // s.y * c.x * c.z + s.x * s.z * c.y,
                // s.z * c.x * c.y - s.x * s.y * c.z,
                // c.x * c.y * c.z - s.y * s.z * s.x
                fp4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * fp4(c.xyz, s.x) * fp4(fp.one, fp.one, fp.minusOne, fp.minusOne)
                );
        }

        /// <summary>
        /// Returns a fpQuaternion constructed by first performing a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fpQuaternion representing the Euler angle rotation in y-x-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion EulerYXZ(fp3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateX(xyz.x), rotateY(xyz.y)));
            fp3 s, c;
            sincos(fp.half * xyz, out s, out c);
            return fpQuaternion(
                // s.x * c.y * c.z - s.y * s.z * c.x,
                // s.y * c.x * c.z + s.x * s.z * c.y,
                // s.z * c.x * c.y + s.x * s.y * c.z,
                // c.x * c.y * c.z - s.y * s.z * s.x
                fp4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * fp4(c.xyz, s.x) * fp4(fp.minusOne, fp.one, fp.one, fp.minusOne)
                );
        }

        /// <summary>
        /// Returns a fpQuaternion constructed by first performing a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fpQuaternion representing the Euler angle rotation in y-z-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion EulerYZX(fp3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateZ(xyz.z), rotateY(xyz.y)));
            fp3 s, c;
            sincos(fp.half * xyz, out s, out c);
            return fpQuaternion(
                // s.x * c.y * c.z - s.y * s.z * c.x,
                // s.y * c.x * c.z - s.x * s.z * c.y,
                // s.z * c.x * c.y + s.x * s.y * c.z,
                // c.x * c.y * c.z + s.y * s.z * s.x
                fp4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * fp4(c.xyz, s.x) * fp4(fp.minusOne, fp.minusOne, fp.one, fp.one)
                );
        }

        /// <summary>
        /// Returns a fpQuaternion constructed by first performing a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fpQuaternion representing the Euler angle rotation in z-x-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion EulerZXY(fp3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateX(xyz.x), rotateZ(xyz.z)));
            fp3 s, c;
            sincos(fp.half * xyz, out s, out c);
            return fpQuaternion(
                // s.x * c.y * c.z + s.y * s.z * c.x,
                // s.y * c.x * c.z - s.x * s.z * c.y,
                // s.z * c.x * c.y - s.x * s.y * c.z,
                // c.x * c.y * c.z + s.y * s.z * s.x
                fp4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * fp4(c.xyz, s.x) * fp4(fp.one, fp.minusOne, fp.minusOne, fp.one)
                );
        }

        /// <summary>
        /// Returns a fpQuaternion constructed by first performing a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fpQuaternion representing the Euler angle rotation in z-y-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion EulerZYX(fp3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateY(xyz.y), rotateZ(xyz.z)));
            fp3 s, c;
            sincos(fp.half * xyz, out s, out c);
            return fpQuaternion(
                // s.x * c.y * c.z + s.y * s.z * c.x,
                // s.y * c.x * c.z - s.x * s.z * c.y,
                // s.z * c.x * c.y + s.x * s.y * c.z,
                // c.x * c.y * c.z - s.y * s.x * s.z
                fp4(s.xyz, c.x) * c.yxxy * c.zzyz + s.yxxy * s.zzyz * fp4(c.xyz, s.x) * fp4(fp.one, fp.minusOne, fp.one, fp.minusOne)
                );
        }

        /// <summary>
        /// Returns a fpQuaternion constructed by first performing a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fpQuaternion representing the Euler angle rotation in x-y-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion EulerXYZ(fp x, fp y, fp z) { return EulerXYZ(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fpQuaternion constructed by first performing a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fpQuaternion representing the Euler angle rotation in x-z-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion EulerXZY(fp x, fp y, fp z) { return EulerXZY(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fpQuaternion constructed by first performing a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fpQuaternion representing the Euler angle rotation in y-x-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion EulerYXZ(fp x, fp y, fp z) { return EulerYXZ(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fpQuaternion constructed by first performing a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fpQuaternion representing the Euler angle rotation in y-z-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion EulerYZX(fp x, fp y, fp z) { return EulerYZX(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fpQuaternion constructed by first performing a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fpQuaternion representing the Euler angle rotation in z-x-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion EulerZXY(fp x, fp y, fp z) { return EulerZXY(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fpQuaternion constructed by first performing a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fpQuaternion representing the Euler angle rotation in z-y-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion EulerZYX(fp x, fp y, fp z) { return EulerZYX(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fpQuaternion constructed by first performing 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        /// <returns>The fpQuaternion representing the Euler angle rotation in the specified order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion Euler(fp3 xyz, RotationOrder order = RotationOrder.ZXY)
        {
            switch (order)
            {
                case RotationOrder.XYZ:
                    return EulerXYZ(xyz);
                case RotationOrder.XZY:
                    return EulerXZY(xyz);
                case RotationOrder.YXZ:
                    return EulerYXZ(xyz);
                case RotationOrder.YZX:
                    return EulerYZX(xyz);
                case RotationOrder.ZXY:
                    return EulerZXY(xyz);
                case RotationOrder.ZYX:
                    return EulerZYX(xyz);
                default:
                    return identity;
            }
        }

        /// <summary>
        /// Returns a fpQuaternion constructed by first performing 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        /// <returns>The fpQuaternion representing the Euler angle rotation in the specified order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion Euler(fp x, fp y, fp z, RotationOrder order = RotationOrder.Default)
        {
            return Euler(fp3(x, y, z), order);
        }

        /// <summary>Returns a fpQuaternion that rotates around the x-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the x-axis towards the origin in radians.</param>
        /// <returns>The fpQuaternion representing a rotation around the x-axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion RotateX(fp angle)
        {
            fp sina, cosa;
            fpmath.sincos(fp.half * angle, out sina, out cosa);
            return fpQuaternion(sina, fp.zero, fp.zero, cosa);
        }

        /// <summary>Returns a fpQuaternion that rotates around the y-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the y-axis towards the origin in radians.</param>
        /// <returns>The fpQuaternion representing a rotation around the y-axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion RotateY(fp angle)
        {
            fp sina, cosa;
            fpmath.sincos(fp.half * angle, out sina, out cosa);
            return fpQuaternion(fp.zero, sina, fp.zero, cosa);
        }

        /// <summary>Returns a fpQuaternion that rotates around the z-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the z-axis towards the origin in radians.</param>
        /// <returns>The fpQuaternion representing a rotation around the z-axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion RotateZ(fp angle)
        {
            fp sina, cosa;
            fpmath.sincos(fp.half * angle, out sina, out cosa);
            return fpQuaternion(fp.zero, fp.zero, sina, cosa);
        }

        /// <summary>
        /// Returns a fpQuaternion view rotation given a unit length forward vector and a unit length up vector.
        /// The two input vectors are assumed to be unit length and not collinear.
        /// If these assumptions are not met use fp3x3.LookRotationSafe instead.
        /// </summary>
        /// <param name="forward">The view forward direction.</param>
        /// <param name="up">The view up direction.</param>
        /// <returns>The fpQuaternion view rotation.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion LookRotation(fp3 forward, fp3 up)
        {
            fp3 t = normalize(cross(up, forward));
            return fpQuaternion(fp3x3(t, cross(forward, t), forward));
        }

        /// <summary>
        /// Returns a fpQuaternion view rotation given a forward vector and an up vector.
        /// The two input vectors are not assumed to be unit length.
        /// If the magnitude of either of the vectors is so extreme that the calculation cannot be carried out reliably or the vectors are collinear,
        /// the identity will be returned instead.
        /// </summary>
        /// <param name="forward">The view forward direction.</param>
        /// <param name="up">The view up direction.</param>
        /// <returns>The fpQuaternion view rotation or the identity fpQuaternion.</returns>
        public static fpquaternion LookRotationSafe(fp3 forward, fp3 up)
        {
            fp forwardLengthSq = dot(forward, forward);
            fp upLengthSq = dot(up, up);

            forward *= rsqrt(forwardLengthSq);
            up *= rsqrt(upLengthSq);

            fp3 t = cross(up, forward);
            fp tLengthSq = dot(t, t);
            t *= rsqrt(tLengthSq);

            fp mn = min(min(forwardLengthSq, upLengthSq), tLengthSq);
            fp mx = max(max(forwardLengthSq, upLengthSq), tLengthSq);

            bool accept = mn > (fp)1e-35f && mx < (fp)1e35f && isfinite(forwardLengthSq) && isfinite(upLengthSq) && isfinite(tLengthSq);
            return fpQuaternion(select(fp4(fp.zero, fp.zero, fp.zero, fp.one), fpQuaternion(fp3x3(t, cross(forward, t),forward)).value, accept));
        }

        /// <summary>Returns true if the fpQuaternion is equal to a given fpQuaternion, false otherwise.</summary>
        /// <param name="x">The fpQuaternion to compare with.</param>
        /// <returns>True if the fpQuaternion is equal to the input, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Equals(fpquaternion x) { return value.x == x.value.x && value.y == x.value.y && value.z == x.value.z && value.w == x.value.w; }

        /// <summary>Returns whether true if the fpQuaternion is equal to a given fpQuaternion, false otherwise.</summary>
        /// <param name="x">The object to compare with.</param>
        /// <returns>True if the fpQuaternion is equal to the input, false otherwise.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool Equals(object x) { return x is fpquaternion converted && Equals(converted); }

        /// <summary>Returns a hash code for the fpQuaternion.</summary>
        /// <returns>The hash code of the fpQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int GetHashCode() { return (int)fpmath.hash(this); }

        /// <summary>Returns a string representation of the fpQuaternion.</summary>
        /// <returns>The string representation of the fpQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString()
        {
            return string.Format("fpQuaternion({0}f, {1}f, {2}f, {3}f)", value.x, value.y, value.z, value.w);
        }

        /// <summary>Returns a string representation of the fpQuaternion using a specified format and culture-specific format information.</summary>
        /// <param name="format">The format string.</param>
        /// <param name="formatProvider">The format provider to use during string formatting.</param>
        /// <returns>The formatted string representation of the fpQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public string ToString(string format, IFormatProvider formatProvider)
        {
            return string.Format("fpQuaternion({0}f, {1}f, {2}f, {3}f)", value.x.ToString(format, formatProvider), value.y.ToString(format, formatProvider), value.z.ToString(format, formatProvider), value.w.ToString(format, formatProvider));
        }
    }

    public static partial class fpmath
    {
        /// <summary>Returns a fpQuaternion constructed from four fp values.</summary>
        /// <param name="x">The x component of the fpQuaternion.</param>
        /// <param name="y">The y component of the fpQuaternion.</param>
        /// <param name="z">The z component of the fpQuaternion.</param>
        /// <param name="w">The w component of the fpQuaternion.</param>
        /// <returns>The fpQuaternion constructed from individual components.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion fpQuaternion(fp x, fp y, fp z, fp w) { return new fpquaternion(x, y, z, w); }

        /// <summary>Returns a fpQuaternion constructed from a fp4 vector.</summary>
        /// <param name="value">The fp4 containing the components of the fpQuaternion.</param>
        /// <returns>The fpQuaternion constructed from a fp4.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion fpQuaternion(fp4 value) { return new fpquaternion(value); }

        /// <summary>Returns a unit fpQuaternion constructed from a fp3x3 rotation matrix. The matrix must be orthonormal.</summary>
        /// <param name="m">The fp3x3 rotation matrix.</param>
        /// <returns>The fpQuaternion constructed from a fp3x3 matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion fpQuaternion(fp3x3 m) { return new fpquaternion(m); }

        /// <summary>Returns a unit fpQuaternion constructed from a fp4x4 matrix. The matrix must be orthonormal.</summary>
        /// <param name="m">The fp4x4 matrix (must be orthonormal).</param>
        /// <returns>The fpQuaternion constructed from a fp4x4 matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion fpQuaternion(fp4x4 m) { return new fpquaternion(m); }

       /// <summary>Returns the conjugate of a fpQuaternion value.</summary>
       /// <param name="q">The fpQuaternion to conjugate.</param>
       /// <returns>The conjugate of the input fpQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion conjugate(fpquaternion q)
        {
            return fpQuaternion(q.value * fp4(fp.minusOne, fp.minusOne, fp.minusOne, fp.one));
        }

       /// <summary>Returns the inverse of a fpQuaternion value.</summary>
       /// <param name="q">The fpQuaternion to invert.</param>
       /// <returns>The fpQuaternion inverse of the input fpQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion inverse(fpquaternion q)
        {
            fp4 x = q.value;
            return fpQuaternion(rcp(dot(x, x)) * x * fp4(fp.minusOne, fp.minusOne, fp.minusOne, fp.one));
        }

        /// <summary>Returns the dot product of two quaternions.</summary>
        /// <param name="a">The first fpQuaternion.</param>
        /// <param name="b">The second fpQuaternion.</param>
        /// <returns>The dot product of two quaternions.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp dot(fpquaternion a, fpquaternion b)
        {
            return dot(a.value, b.value);
        }

        /// <summary>Returns the length of a fpQuaternion.</summary>
        /// <param name="q">The input fpQuaternion.</param>
        /// <returns>The length of the input fpQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp length(fpquaternion q)
        {
            return sqrt(dot(q.value, q.value));
        }

        /// <summary>Returns the squared length of a fpQuaternion.</summary>
        /// <param name="q">The input fpQuaternion.</param>
        /// <returns>The length squared of the input fpQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp lengthsq(fpquaternion q)
        {
            return dot(q.value, q.value);
        }

        /// <summary>Returns a normalized version of a fpQuaternion q by scaling it by 1 / length(q).</summary>
        /// <param name="q">The fpQuaternion to normalize.</param>
        /// <returns>The normalized fpQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion normalize(fpquaternion q)
        {
            fp4 x = q.value;
            return fpQuaternion(rsqrt(dot(x, x)) * x);
        }

        /// <summary>
        /// Returns a safe normalized version of the q by scaling it by 1 / length(q).
        /// Returns the identity when 1 / length(q) does not produce a finite number.
        /// </summary>
        /// <param name="q">The fpQuaternion to normalize.</param>
        /// <returns>The normalized fpQuaternion or the identity fpQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion normalizesafe(fpquaternion q)
        {
            fp4 x = q.value;
            fp len = fpmath.dot(x, x);
            return fpQuaternion(fpmath.select(FixedPoint.fpquaternion.identity.value, x * fpmath.rsqrt(len), len > FLT_MIN_NORMAL));
        }

        /// <summary>
        /// Returns a safe normalized version of the q by scaling it by 1 / length(q).
        /// Returns the given default value when 1 / length(q) does not produce a finite number.
        /// </summary>
        /// <param name="q">The fpQuaternion to normalize.</param>
        /// <param name="defaultvalue">The default value.</param>
        /// <returns>The normalized fpQuaternion or the default value.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion normalizesafe(fpquaternion q, fpquaternion defaultvalue)
        {
            fp4 x = q.value;
            fp len = fpmath.dot(x, x);
            return fpQuaternion(fpmath.select(defaultvalue.value, x * fpmath.rsqrt(len), len > FLT_MIN_NORMAL));
        }

        /// <summary>Returns the natural exponent of a fpQuaternion. Assumes w is zero.</summary>
        /// <param name="q">The fpQuaternion with w component equal to zero.</param>
        /// <returns>The natural exponent of the input fpQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion unitexp(fpquaternion q)
        {
            fp v_rcp_len = rsqrt(dot(q.value.xyz, q.value.xyz));
            fp v_len = rcp(v_rcp_len);
            fp sin_v_len, cos_v_len;
            sincos(v_len, out sin_v_len, out cos_v_len);
            return fpQuaternion(fp4(q.value.xyz * v_rcp_len * sin_v_len, cos_v_len));
        }

        /// <summary>Returns the natural exponent of a fpQuaternion.</summary>
        /// <param name="q">The fpQuaternion.</param>
        /// <returns>The natural exponent of the input fpQuaternion.</returns>
        // [MethodImpl(MethodImplOptions.AggressiveInlining)]
        // public static fpQuaternion exp(fpQuaternion q)
        // {
        //     fp v_rcp_len = rsqrt(dot(q.value.xyz, q.value.xyz));
        //     fp v_len = rcp(v_rcp_len);
        //     fp sin_v_len, cos_v_len;
        //     sincos(v_len, out sin_v_len, out cos_v_len);
        //     return fpQuaternion(fp4(q.value.xyz * v_rcp_len * sin_v_len, cos_v_len) * exp(q.value.w));
        // }

        /// <summary>Returns the natural logarithm of a unit length fpQuaternion.</summary>
        /// <param name="q">The unit length fpQuaternion.</param>
        /// <returns>The natural logarithm of the unit length fpQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion unitlog(fpquaternion q)
        {
            fp w = clamp(q.value.w, fp.minusOne, fp.one);
            fp s = fp.Acos(w) * rsqrt(fp.one - w*w);
            return fpQuaternion(fp4(q.value.xyz * s, fp.zero));
        }

        /// <summary>Returns the natural logarithm of a fpQuaternion.</summary>
        /// <param name="q">The fpQuaternion.</param>
        /// <returns>The natural logarithm of the input fpQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion log(fpquaternion q)
        {
            fp v_len_sq = dot(q.value.xyz, q.value.xyz);
            fp q_len_sq = v_len_sq + q.value.w*q.value.w;

            fp s = fp.Acos(clamp(q.value.w * rsqrt(q_len_sq), fp.minusOne, fp.one)) * rsqrt(v_len_sq);
            return fpQuaternion(fp4(q.value.xyz * s, fp.half * log(q_len_sq)));
        }

        /// <summary>Returns the result of transforming the fpQuaternion b by the fpQuaternion a.</summary>
        /// <param name="a">The fpQuaternion on the left.</param>
        /// <param name="b">The fpQuaternion on the right.</param>
        /// <returns>The result of transforming fpQuaternion b by the fpQuaternion a.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion mul(fpquaternion a, fpquaternion b)
        {
            return fpQuaternion(a.value.wwww * b.value + (a.value.xyzx * b.value.wwwx + a.value.yzxy * b.value.zxyy) * fp4(fp.one, fp.one, fp.one, fp.minusOne) - a.value.zxyz * b.value.yzxz);
        }

        /// <summary>Returns the result of transforming a vector by a fpQuaternion.</summary>
        /// <param name="q">The fpQuaternion transformation.</param>
        /// <param name="v">The vector to transform.</param>
        /// <returns>The transformation of vector v by fpQuaternion q.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3 mul(fpquaternion q, fp3 v)
        {
            fp3 t = 2 * cross(q.value.xyz, v);
            return v + q.value.w * t + cross(q.value.xyz, t);
        }

        /// <summary>Returns the result of rotating a vector by a unit fpQuaternion.</summary>
        /// <param name="q">The fpQuaternion rotation.</param>
        /// <param name="v">The vector to rotate.</param>
        /// <returns>The rotation of vector v by fpQuaternion q.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3 rotate(fpquaternion q, fp3 v)
        {
            fp3 t = 2 * cross(q.value.xyz, v);
            return v + q.value.w * t + cross(q.value.xyz, t);
        }

        /// <summary>Returns the result of a normalized linear interpolation between two quaternions q1 and a2 using an interpolation parameter t.</summary>
        /// <remarks>
        /// Prefer to use this over slerp() when you know the distance between q1 and q2 is small. This can be much
        /// higher performance due to avoiding trigonometric function evaluations that occur in slerp().
        /// </remarks>
        /// <param name="q1">The first fpQuaternion.</param>
        /// <param name="q2">The second fpQuaternion.</param>
        /// <param name="t">The interpolation parameter.</param>
        /// <returns>The normalized linear interpolation of two quaternions.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion nlerp(fpquaternion q1, fpquaternion q2, fp t)
        {
            fp dt = dot(q1, q2);
            if(dt < fp.zero)
            {
                q2.value = -q2.value;
            }

            return normalize(fpQuaternion(lerp(q1.value, q2.value, t)));
        }

        /// <summary>Returns the result of a spherical interpolation between two quaternions q1 and a2 using an interpolation parameter t.</summary>
        /// <param name="q1">The first fpQuaternion.</param>
        /// <param name="q2">The second fpQuaternion.</param>
        /// <param name="t">The interpolation parameter.</param>
        /// <returns>The spherical linear interpolation of two quaternions.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fpquaternion slerp(fpquaternion q1, fpquaternion q2, fp t)
        {
            fp dt = dot(q1, q2);
            if (dt < fp.zero)
            {
                dt = -dt;
                q2.value = -q2.value;
            }

            if (dt < (fp)0.9995f)
            {
                fp angle = fp.Acos(dt);
                fp s = rsqrt(fp.one - dt * dt);    // fp.one / sin(angle)
                fp w1 = sin(angle * (fp.one - t)) * s;
                fp w2 = sin(angle * t) * s;
                return fpQuaternion(q1.value * w1 + q2.value * w2);
            }
            else
            {
                // if the angle is small, use linear interpolation
                return nlerp(q1, q2, t);
            }
        }

        /// <summary>Returns a uint hash code of a fpQuaternion.</summary>
        /// <param name="q">The fpQuaternion to hash.</param>
        /// <returns>The hash code for the input fpQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint hash(fpquaternion q)
        {
            return hash(q.value);
        }

        /// <summary>
        /// Returns a uint4 vector hash code of a fpQuaternion.
        /// When multiple elements are to be hashes together, it can more efficient to calculate and combine wide hash
        /// that are only reduced to a narrow uint hash at the very end instead of at every step.
        /// </summary>
        /// <param name="q">The fpQuaternion to hash.</param>
        /// <returns>The uint4 vector hash code of the input fpQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static uint4 hashwide(fpquaternion q)
        {
            return hashwide(q.value);
        }


        /// <summary>
        /// Transforms the forward vector by a fpQuaternion.
        /// </summary>
        /// <param name="q">The fpQuaternion transformation.</param>
        /// <returns>The forward vector transformed by the input fpQuaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3 forward(fpquaternion q) { return mul(q, fp3(0, 0, 1)); }  // for compatibility
    }
}
