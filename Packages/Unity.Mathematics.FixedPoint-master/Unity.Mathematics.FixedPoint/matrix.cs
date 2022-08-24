using System.Runtime.CompilerServices;
using static Unity.Mathematics.math;
using static Unity.Mathematics.FixedPoint.fpmath;

namespace Unity.Mathematics.FixedPoint
{
    public partial struct fp2x2
    {
        /// <summary>
        /// Computes a fp2x2 matrix representing a counter-clockwise rotation by an angle in radians.
        /// </summary>
        /// <remarks>
        /// A positive rotation angle will produce a counter-clockwise rotation and a negative rotation angle will
        /// produce a clockwise rotation.
        /// </remarks>
        /// <param name="angle">Rotation angle in radians.</param>
        /// <returns>Returns the 2x2 rotation matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp2x2 Rotate(fp angle)
        {
            fp s, c;
            sincos(angle, out s, out c);
            return fp2x2(c, -s,
                            s,  c);
        }

        /// <summary>Returns a fp2x2 matrix representing a uniform scaling of both axes by s.</summary>
        /// <param name="s">The scaling factor.</param>
        /// <returns>The fp2x2 matrix representing uniform scale by s.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp2x2 Scale(fp s)
        {
            return fp2x2(s,    fp.zero,
                            fp.zero, s);
        }

        /// <summary>Returns a fp2x2 matrix representing a non-uniform axis scaling by x and y.</summary>
        /// <param name="x">The x-axis scaling factor.</param>
        /// <param name="y">The y-axis scaling factor.</param>
        /// <returns>The fp2x2 matrix representing a non-uniform scale.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp2x2 Scale(fp x, fp y)
        {
            return fp2x2(x,    fp.zero,
                            fp.zero, y);
        }

        /// <summary>Returns a fp2x2 matrix representing a non-uniform axis scaling by the components of the fp2 vector v.</summary>
        /// <param name="v">The fp2 containing the x and y axis scaling factors.</param>
        /// <returns>The fp2x2 matrix representing a non-uniform scale.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp2x2 Scale(fp2 v)
        {
            return Scale(v.x, v.y);
        }
    }

    public partial struct fp3x3
    {
        /// <summary>
        /// Constructs a fp3x3 from the upper left 3x3 of a fp4x4.
        /// </summary>
        /// <param name="f4x4"><see cref="fp4x4"/> to extract a fp3x3 from.</param>
        public fp3x3(fp4x4 f4x4)
        {
            c0 = f4x4.c0.xyz;
            c1 = f4x4.c1.xyz;
            c2 = f4x4.c2.xyz;
        }

        /// <summary>Constructs a fp3x3 matrix from a unit quaternion.</summary>
        /// <param name="q">The quaternion rotation.</param>
        public fp3x3(fpquaternion q)
        {
            fp4 v = q.value;
            fp4 v2 = v + v;

            uint3 npn = uint3(0x80000000, 0x00000000, 0x80000000);
            uint3 nnp = uint3(0x80000000, 0x80000000, 0x00000000);
            uint3 pnn = uint3(0x00000000, 0x80000000, 0x80000000);
            //TODO asfloat
            c0 = v2.y * (fp3)(asuint(v.yxw) ^ npn) - v2.z * (fp3)(asuint(v.zwx) ^ pnn) + fp3(fp.one, fp.zero, fp.zero);
            c1 = v2.z * (fp3)(asuint(v.wzy) ^ nnp) - v2.x * (fp3)(asuint(v.yxw) ^ npn) + fp3(fp.zero, fp.one, fp.zero);
            c2 = v2.x * (fp3)(asuint(v.zwx) ^ pnn) - v2.y * (fp3)(asuint(v.wzy) ^ nnp) + fp3(fp.zero, fp.zero, fp.one);
        }

        /// <summary>
        /// Returns a fp3x3 matrix representing a rotation around a unit axis by an angle in radians.
        /// The rotation direction is clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="axis">The rotation axis.</param>
        /// <param name="angle">The angle of rotation in radians.</param>
        /// <returns>The fp3x3 matrix representing the rotation around an axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 AxisAngle(fp3 axis, fp angle)
        {
            fp sina, cosa;
            fpmath.sincos(angle, out sina, out cosa);

            fp3 u = axis;
            fp3 u_yzx = u.yzx;
            fp3 u_zxy = u.zxy;
            fp3 u_inv_cosa = u - u * cosa;  // u * (fp.one - cosa);
            fp4 t = fp4(u * sina, cosa);

            uint3 ppn = uint3(0x00000000, 0x00000000, 0x80000000);
            uint3 npp = uint3(0x80000000, 0x00000000, 0x00000000);
            uint3 pnp = uint3(0x00000000, 0x80000000, 0x00000000);

            //TODO asfloat
            return fp3x3(
                u.x * u_inv_cosa + (fp3)(asuint(t.wzy) ^ ppn),
                u.y * u_inv_cosa + (fp3)(asuint(t.zwx) ^ npp),
                u.z * u_inv_cosa + (fp3)(asuint(t.yxw) ^ pnp)
                );
            /*
            return fp3x3(
                cosa + u.x * u.x * (fp.one - cosa),       u.y * u.x * (fp.one - cosa) - u.z * sina, u.z * u.x * (fp.one - cosa) + u.y * sina,
                u.x * u.y * (fp.one - cosa) + u.z * sina, cosa + u.y * u.y * (fp.one - cosa),       u.y * u.z * (fp.one - cosa) - u.x * sina,
                u.x * u.z * (fp.one - cosa) - u.y * sina, u.y * u.z * (fp.one - cosa) + u.x * sina, cosa + u.z * u.z * (fp.one - cosa)
                );
                */
        }

        /// <summary>
        /// Returns a fp3x3 rotation matrix constructed by first performing a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fp3x3 rotation matrix representing the rotation by Euler angles in x-y-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 EulerXYZ(fp3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateY(xyz.y), rotateX(xyz.x)));
            fp3 s, c;
            sincos(xyz, out s, out c);
            return fp3x3(
                c.y * c.z,  c.z * s.x * s.y - c.x * s.z,    c.x * c.z * s.y + s.x * s.z,
                c.y * s.z,  c.x * c.z + s.x * s.y * s.z,    c.x * s.y * s.z - c.z * s.x,
                -s.y,       c.y * s.x,                      c.x * c.y
                );
        }

        /// <summary>
        /// Returns a fp3x3 rotation matrix constructed by first performing a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fp3x3 rotation matrix representing the rotation by Euler angles in x-z-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 EulerXZY(fp3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateZ(xyz.z), rotateX(xyz.x))); }
            fp3 s, c;
            sincos(xyz, out s, out c);
            return fp3x3(
                c.y * c.z,  s.x * s.y - c.x * c.y * s.z,    c.x * s.y + c.y * s.x * s.z,
                s.z,        c.x * c.z,                      -c.z * s.x,
                -c.z * s.y, c.y * s.x + c.x * s.y * s.z,    c.x * c.y - s.x * s.y * s.z
                );
        }

        /// <summary>
        /// Returns a fp3x3 rotation matrix constructed by first performing a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fp3x3 rotation matrix representing the rotation by Euler angles in y-x-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 EulerYXZ(fp3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateX(xyz.x), rotateY(xyz.y)));
            fp3 s, c;
            sincos(xyz, out s, out c);
            return fp3x3(
                c.y * c.z - s.x * s.y * s.z,    -c.x * s.z, c.z * s.y + c.y * s.x * s.z,
                c.z * s.x * s.y + c.y * s.z,    c.x * c.z,  s.y * s.z - c.y * c.z * s.x,
                -c.x * s.y,                     s.x,        c.x * c.y
                );
        }

        /// <summary>
        /// Returns a fp3x3 rotation matrix constructed by first performing a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fp3x3 rotation matrix representing the rotation by Euler angles in y-z-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 EulerYZX(fp3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateZ(xyz.z), rotateY(xyz.y)));
            fp3 s, c;
            sincos(xyz, out s, out c);
            return fp3x3(
                c.y * c.z,                      -s.z,       c.z * s.y,
                s.x * s.y + c.x * c.y * s.z,    c.x * c.z,  c.x * s.y * s.z - c.y * s.x,
                c.y * s.x * s.z - c.x * s.y,    c.z * s.x,  c.x * c.y + s.x * s.y * s.z
                );
        }

        /// <summary>
        /// Returns a fp3x3 rotation matrix constructed by first performing a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fp3x3 rotation matrix representing the rotation by Euler angles in z-x-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 EulerZXY(fp3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateX(xyz.x), rotateZ(xyz.z)));
            fp3 s, c;
            sincos(xyz, out s, out c);
            return fp3x3(
                c.y * c.z + s.x * s.y * s.z,    c.z * s.x * s.y - c.y * s.z,    c.x * s.y,
                c.x * s.z,                      c.x * c.z,                      -s.x,
                c.y * s.x * s.z - c.z * s.y,    c.y * c.z * s.x + s.y * s.z,    c.x * c.y
                );
        }

        /// <summary>
        /// Returns a fp3x3 rotation matrix constructed by first performing a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fp3x3 rotation matrix representing the rotation by Euler angles in z-y-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 EulerZYX(fp3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateY(xyz.y), rotateZ(xyz.z)));
            fp3 s, c;
            sincos(xyz, out s, out c);
            return fp3x3(
                c.y * c.z,                      -c.y * s.z,                     s.y,
                c.z * s.x * s.y + c.x * s.z,    c.x * c.z - s.x * s.y * s.z,    -c.y * s.x,
                s.x * s.z - c.x * c.z * s.y,    c.z * s.x + c.x * s.y * s.z,    c.x * c.y
                );
        }

        /// <summary>
        /// Returns a fp3x3 rotation matrix constructed by first performing a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fp3x3 rotation matrix representing the rotation by Euler angles in x-y-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 EulerXYZ(fp x, fp y, fp z) { return EulerXYZ(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fp3x3 rotation matrix constructed by first performing a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fp3x3 rotation matrix representing the rotation by Euler angles in x-z-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 EulerXZY(fp x, fp y, fp z) { return EulerXZY(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fp3x3 rotation matrix constructed by first performing a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fp3x3 rotation matrix representing the rotation by Euler angles in y-x-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 EulerYXZ(fp x, fp y, fp z) { return EulerYXZ(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fp3x3 rotation matrix constructed by first performing a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fp3x3 rotation matrix representing the rotation by Euler angles in y-z-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 EulerYZX(fp x, fp y, fp z) { return EulerYZX(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fp3x3 rotation matrix constructed by first performing a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fp3x3 rotation matrix representing the rotation by Euler angles in z-x-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 EulerZXY(fp x, fp y, fp z) { return EulerZXY(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fp3x3 rotation matrix constructed by first performing a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fp3x3 rotation matrix representing the rotation by Euler angles in z-y-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 EulerZYX(fp x, fp y, fp z) { return EulerZYX(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fp3x3 rotation matrix constructed by first performing 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        /// <returns>The fp3x3 rotation matrix representing the rotation by Euler angles in the given order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 Euler(fp3 xyz, RotationOrder order = RotationOrder.Default)
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
                    return fp3x3.identity;
            }
        }

        /// <summary>
        /// Returns a fp3x3 rotation matrix constructed by first performing 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        /// <returns>The fp3x3 rotation matrix representing the rotation by Euler angles in the given order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 Euler(fp x, fp y, fp z, RotationOrder order = RotationOrder.Default)
        {
            return Euler(fp3(x, y, z), order);
        }

        /// <summary>Returns a fp3x3 matrix that rotates around the x-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the x-axis towards the origin in radians.</param>
        /// <returns>The fp3x3 rotation matrix representing a rotation around the x-axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 RotateX(fp angle)
        {
            // {{1, 0, 0}, {0, c_0, -s_0}, {0, s_0, c_0}}
            fp s, c;
            sincos(angle, out s, out c);
            return fp3x3(fp.one, fp.zero, fp.zero,
                            fp.zero, c,    -s,
                            fp.zero, s,    c);
        }

        /// <summary>Returns a fp3x3 matrix that rotates around the y-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the y-axis towards the origin in radians.</param>
        /// <returns>The fp3x3 rotation matrix representing a rotation around the y-axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 RotateY(fp angle)
        {
            // {{c_1, 0, s_1}, {0, 1, 0}, {-s_1, 0, c_1}}
            fp s, c;
            sincos(angle, out s, out c);
            return fp3x3(c,    fp.zero, s,
                            fp.zero, fp.one, fp.zero,
                            -s,   fp.zero, c);
        }

        /// <summary>Returns a fp3x3 matrix that rotates around the z-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the z-axis towards the origin in radians.</param>
        /// <returns>The fp3x3 rotation matrix representing a rotation around the z-axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 RotateZ(fp angle)
        {
            // {{c_2, -s_2, 0}, {s_2, c_2, 0}, {0, 0, 1}}
            fp s, c;
            sincos(angle, out s, out c);
            return fp3x3(c,    -s,   fp.zero,
                            s,    c,    fp.zero,
                            fp.zero, fp.zero, fp.one);
        }

        /// <summary>Returns a fp3x3 matrix representing a uniform scaling of all axes by s.</summary>
        /// <param name="s">The uniform scaling factor.</param>
        /// <returns>The fp3x3 matrix representing a uniform scale.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 Scale(fp s)
        {
            return fp3x3(s,    fp.zero, fp.zero,
                            fp.zero, s,    fp.zero,
                            fp.zero, fp.zero, s);
        }

        /// <summary>Returns a fp3x3 matrix representing a non-uniform axis scaling by x, y and z.</summary>
        /// <param name="x">The x-axis scaling factor.</param>
        /// <param name="y">The y-axis scaling factor.</param>
        /// <param name="z">The z-axis scaling factor.</param>
        /// <returns>The fp3x3 rotation matrix representing a non-uniform scale.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 Scale(fp x, fp y, fp z)
        {
            return fp3x3(x,    fp.zero, fp.zero,
                            fp.zero, y,    fp.zero,
                            fp.zero, fp.zero, z);
        }

        /// <summary>Returns a fp3x3 matrix representing a non-uniform axis scaling by the components of the fp3 vector v.</summary>
        /// <param name="v">The vector containing non-uniform scaling factors.</param>
        /// <returns>The fp3x3 rotation matrix representing a non-uniform scale.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 Scale(fp3 v)
        {
            return Scale(v.x, v.y, v.z);
        }

        /// <summary>
        /// Returns a fp3x3 view rotation matrix given a unit length forward vector and a unit length up vector.
        /// The two input vectors are assumed to be unit length and not collinear.
        /// If these assumptions are not met use fp3x3.LookRotationSafe instead.
        /// </summary>
        /// <param name="forward">The forward vector to align the center of view with.</param>
        /// <param name="up">The up vector to point top of view toward.</param>
        /// <returns>The fp3x3 view rotation matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 LookRotation(fp3 forward, fp3 up)
        {
            fp3 t = normalize(cross(up, forward));
            return fp3x3(t, cross(forward, t), forward);
        }

        /// <summary>
        /// Returns a fp3x3 view rotation matrix given a forward vector and an up vector.
        /// The two input vectors are not assumed to be unit length.
        /// If the magnitude of either of the vectors is so extreme that the calculation cannot be carried out reliably or the vectors are collinear,
        /// the identity will be returned instead.
        /// </summary>
        /// <param name="forward">The forward vector to align the center of view with.</param>
        /// <param name="up">The up vector to point top of view toward.</param>
        /// <returns>The fp3x3 view rotation matrix or the identity matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 LookRotationSafe(fp3 forward, fp3 up)
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

            const uint bigValue = 0x799a130c;
            const uint smallValue = 0x0554ad2e;

            bool accept = mn > fp.FromRaw(smallValue) && mx < fp.FromRaw(bigValue) && isfinite(forwardLengthSq) && isfinite(upLengthSq) && isfinite(tLengthSq);
            return fp3x3(
                select(fp3(fp.one, fp.zero, fp.zero), t, accept),
                select(fp3(fp.zero, fp.one, fp.zero), cross(forward, t), accept),
                select(fp3(fp.zero, fp.zero, fp.one), forward, accept));
        }

        /// <summary>
        /// Converts a fp4x4 to a fp3x3.
        /// </summary>
        /// <param name="f4x4">The fp4x4 to convert to a fp3x3.</param>
        /// <returns>The fp3x3 constructed from the upper left 3x3 of the input fp4x4 matrix.</returns>
        public static explicit operator fp3x3(fp4x4 f4x4) => new fp3x3(f4x4);
    }

    public partial struct fp4x4
    {
        /// <summary>Constructs a fp4x4 from a fp3x3 rotation matrix and a fp3 translation vector.</summary>
        /// <param name="rotation">The fp3x3 rotation matrix.</param>
        /// <param name="translation">The translation vector.</param>
        public fp4x4(fp3x3 rotation, fp3 translation)
        {
            c0 = fp4(rotation.c0, fp.zero);
            c1 = fp4(rotation.c1, fp.zero);
            c2 = fp4(rotation.c2, fp.zero);
            c3 = fp4(translation, fp.one);
        }

        /// <summary>Constructs a fp4x4 from a quaternion and a fp3 translation vector.</summary>
        /// <param name="rotation">The quaternion rotation.</param>
        /// <param name="translation">The translation vector.</param>
        public fp4x4(fpquaternion rotation, fp3 translation)
        {
            fp3x3 rot = fp3x3(rotation);
            c0 = fp4(rot.c0, fp.zero);
            c1 = fp4(rot.c1, fp.zero);
            c2 = fp4(rot.c2, fp.zero);
            c3 = fp4(translation, fp.one);
        }

        /// <summary>Constructs a fp4x4 from a RigidTransform.</summary>
        /// <param name="transform">The RigidTransform.</param>
        public fp4x4(FpRigidTransform transform)
        {
            fp3x3 rot = fp3x3(transform.rot);
            c0 = fp4(rot.c0, fp.zero);
            c1 = fp4(rot.c1, fp.zero);
            c2 = fp4(rot.c2, fp.zero);
            c3 = fp4(transform.pos, fp.one);
        }

        /// <summary>
        /// Returns a fp4x4 matrix representing a rotation around a unit axis by an angle in radians.
        /// The rotation direction is clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="axis">The axis of rotation.</param>
        /// <param name="angle">The angle of rotation in radians.</param>
        /// <returns>The fp4x4 matrix representing the rotation about an axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 AxisAngle(fp3 axis, fp angle)
        {
            fp sina, cosa;
            fpmath.sincos(angle, out sina, out cosa);

            fp4 u = fp4(axis, fp.zero);
            fp4 u_yzx = u.yzxx;
            fp4 u_zxy = u.zxyx;
            fp4 u_inv_cosa = u - u * cosa;  // u * (fp.one - cosa);
            fp4 t = fp4(u.xyz * sina, cosa);

            uint4 ppnp = uint4(0x00000000, 0x00000000, 0x80000000, 0x00000000);
            uint4 nppp = uint4(0x80000000, 0x00000000, 0x00000000, 0x00000000);
            uint4 pnpp = uint4(0x00000000, 0x80000000, 0x00000000, 0x00000000);
            uint4 mask = uint4(0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x00000000);

            //TODO asfloat
            return fp4x4(
                u.x * u_inv_cosa + (fp4)((asuint(t.wzyx) ^ ppnp) & mask),
                u.y * u_inv_cosa + (fp4)((asuint(t.zwxx) ^ nppp) & mask),
                u.z * u_inv_cosa + (fp4)((asuint(t.yxwx) ^ pnpp) & mask),
                fp4(fp.zero, fp.zero, fp.zero, fp.one)
                );

        }

        /// <summary>
        /// Returns a fp4x4 rotation matrix constructed by first performing a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fp4x4 rotation matrix of the Euler angle rotation in x-y-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 EulerXYZ(fp3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateY(xyz.y), rotateX(xyz.x)));
            fp3 s, c;
            sincos(xyz, out s, out c);
            return fp4x4(
                c.y * c.z,  c.z * s.x * s.y - c.x * s.z,    c.x * c.z * s.y + s.x * s.z,    fp.zero,
                c.y * s.z,  c.x * c.z + s.x * s.y * s.z,    c.x * s.y * s.z - c.z * s.x,    fp.zero,
                -s.y,       c.y * s.x,                      c.x * c.y,                      fp.zero,
                fp.zero,       fp.zero,                           fp.zero,                           fp.one
                );
        }

        /// <summary>
        /// Returns a fp4x4 rotation matrix constructed by first performing a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fp4x4 rotation matrix of the Euler angle rotation in x-z-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 EulerXZY(fp3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateZ(xyz.z), rotateX(xyz.x))); }
            fp3 s, c;
            sincos(xyz, out s, out c);
            return fp4x4(
                c.y * c.z,  s.x * s.y - c.x * c.y * s.z,    c.x * s.y + c.y * s.x * s.z,    fp.zero,
                s.z,        c.x * c.z,                      -c.z * s.x,                     fp.zero,
                -c.z * s.y, c.y * s.x + c.x * s.y * s.z,    c.x * c.y - s.x * s.y * s.z,    fp.zero,
                fp.zero,       fp.zero,                           fp.zero,                           fp.one
                );
        }

        /// <summary>
        /// Returns a fp4x4 rotation matrix constructed by first performing a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fp4x4 rotation matrix of the Euler angle rotation in y-x-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 EulerYXZ(fp3 xyz)
        {
            // return mul(rotateZ(xyz.z), mul(rotateX(xyz.x), rotateY(xyz.y)));
            fp3 s, c;
            sincos(xyz, out s, out c);
            return fp4x4(
                c.y * c.z - s.x * s.y * s.z,    -c.x * s.z, c.z * s.y + c.y * s.x * s.z,    fp.zero,
                c.z * s.x * s.y + c.y * s.z,    c.x * c.z,  s.y * s.z - c.y * c.z * s.x,    fp.zero,
                -c.x * s.y,                     s.x,        c.x * c.y,                      fp.zero,
                fp.zero,                           fp.zero,       fp.zero,                           fp.one
                );
        }

        /// <summary>
        /// Returns a fp4x4 rotation matrix constructed by first performing a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fp4x4 rotation matrix of the Euler angle rotation in y-z-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 EulerYZX(fp3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateZ(xyz.z), rotateY(xyz.y)));
            fp3 s, c;
            sincos(xyz, out s, out c);
            return fp4x4(
                c.y * c.z,                      -s.z,       c.z * s.y,                      fp.zero,
                s.x * s.y + c.x * c.y * s.z,    c.x * c.z,  c.x * s.y * s.z - c.y * s.x,    fp.zero,
                c.y * s.x * s.z - c.x * s.y,    c.z * s.x,  c.x * c.y + s.x * s.y * s.z,    fp.zero,
                fp.zero,                           fp.zero,       fp.zero,                           fp.one
                );
        }

        /// <summary>
        /// Returns a fp4x4 rotation matrix constructed by first performing a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fp4x4 rotation matrix of the Euler angle rotation in z-x-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 EulerZXY(fp3 xyz)
        {
            // return mul(rotateY(xyz.y), mul(rotateX(xyz.x), rotateZ(xyz.z)));
            fp3 s, c;
            sincos(xyz, out s, out c);
            return fp4x4(
                c.y * c.z + s.x * s.y * s.z,    c.z * s.x * s.y - c.y * s.z,    c.x * s.y,  fp.zero,
                c.x * s.z,                      c.x * c.z,                      -s.x,       fp.zero,
                c.y * s.x * s.z - c.z * s.y,    c.y * c.z * s.x + s.y * s.z,    c.x * c.y,  fp.zero,
                fp.zero,                           fp.zero,                           fp.zero,       fp.one
                );
        }

        /// <summary>
        /// Returns a fp4x4 rotation matrix constructed by first performing a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <returns>The fp4x4 rotation matrix of the Euler angle rotation in z-y-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 EulerZYX(fp3 xyz)
        {
            // return mul(rotateX(xyz.x), mul(rotateY(xyz.y), rotateZ(xyz.z)));
            fp3 s, c;
            sincos(xyz, out s, out c);
            return fp4x4(
                c.y * c.z,                      -c.y * s.z,                     s.y,        fp.zero,
                c.z * s.x * s.y + c.x * s.z,    c.x * c.z - s.x * s.y * s.z,    -c.y * s.x, fp.zero,
                s.x * s.z - c.x * c.z * s.y,    c.z * s.x + c.x * s.y * s.z,    c.x * c.y,  fp.zero,
                fp.zero,                           fp.zero,                           fp.zero,       fp.one
                );
        }

        /// <summary>
        /// Returns a fp4x4 rotation matrix constructed by first performing a rotation around the x-axis, then the y-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fp4x4 rotation matrix of the Euler angle rotation in x-y-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 EulerXYZ(fp x, fp y, fp z) { return EulerXYZ(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fp4x4 rotation matrix constructed by first performing a rotation around the x-axis, then the z-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fp4x4 rotation matrix of the Euler angle rotation in x-z-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 EulerXZY(fp x, fp y, fp z) { return EulerXZY(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fp4x4 rotation matrix constructed by first performing a rotation around the y-axis, then the x-axis and finally the z-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fp4x4 rotation matrix of the Euler angle rotation in y-x-z order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 EulerYXZ(fp x, fp y, fp z) { return EulerYXZ(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fp4x4 rotation matrix constructed by first performing a rotation around the y-axis, then the z-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fp4x4 rotation matrix of the Euler angle rotation in y-z-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 EulerYZX(fp x, fp y, fp z) { return EulerYZX(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fp4x4 rotation matrix constructed by first performing a rotation around the z-axis, then the x-axis and finally the y-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// This is the default order rotation order in Unity.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fp4x4 rotation matrix of the Euler angle rotation in z-x-y order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 EulerZXY(fp x, fp y, fp z) { return EulerZXY(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fp4x4 rotation matrix constructed by first performing a rotation around the z-axis, then the y-axis and finally the x-axis.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <returns>The fp4x4 rotation matrix of the Euler angle rotation in z-y-x order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 EulerZYX(fp x, fp y, fp z) { return EulerZYX(fp3(x, y, z)); }

        /// <summary>
        /// Returns a fp4x4 constructed by first performing 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="xyz">A fp3 vector containing the rotation angles around the x-, y- and z-axis measures in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        /// <returns>The fp4x4 rotation matrix of the Euler angle rotation in given order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 Euler(fp3 xyz, RotationOrder order = RotationOrder.Default)
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
                    return fp4x4.identity;
            }
        }

        /// <summary>
        /// Returns a fp4x4 rotation matrix constructed by first performing 3 rotations around the principal axes in a given order.
        /// All rotation angles are in radians and clockwise when looking along the rotation axis towards the origin.
        /// When the rotation order is known at compile time, it is recommended for performance reasons to use specific
        /// Euler rotation constructors such as EulerZXY(...).
        /// </summary>
        /// <param name="x">The rotation angle around the x-axis in radians.</param>
        /// <param name="y">The rotation angle around the y-axis in radians.</param>
        /// <param name="z">The rotation angle around the z-axis in radians.</param>
        /// <param name="order">The order in which the rotations are applied.</param>
        /// <returns>The fp4x4 rotation matrix of the Euler angle rotation in given order.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 Euler(fp x, fp y, fp z, RotationOrder order = RotationOrder.Default)
        {
            return Euler(fp3(x, y, z), order);
        }

        /// <summary>Returns a fp4x4 matrix that rotates around the x-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the x-axis towards the origin in radians.</param>
        /// <returns>The fp4x4 rotation matrix that rotates around the x-axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 RotateX(fp angle)
        {
            // {{1, 0, 0}, {0, c_0, -s_0}, {0, s_0, c_0}}
            fp s, c;
            sincos(angle, out s, out c);
            return fp4x4(fp.one, fp.zero, fp.zero, fp.zero,
                            fp.zero, c,    -s,   fp.zero,
                            fp.zero, s,    c,    fp.zero,
                            fp.zero, fp.zero, fp.zero, fp.one);

        }

        /// <summary>Returns a fp4x4 matrix that rotates around the y-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the y-axis towards the origin in radians.</param>
        /// <returns>The fp4x4 rotation matrix that rotates around the y-axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 RotateY(fp angle)
        {
            // {{c_1, 0, s_1}, {0, 1, 0}, {-s_1, 0, c_1}}
            fp s, c;
            sincos(angle, out s, out c);
            return fp4x4(c,    fp.zero, s,    fp.zero,
                            fp.zero, fp.one, fp.zero, fp.zero,
                            -s,   fp.zero, c,    fp.zero,
                            fp.zero, fp.zero, fp.zero, fp.one);

        }

        /// <summary>Returns a fp4x4 matrix that rotates around the z-axis by a given number of radians.</summary>
        /// <param name="angle">The clockwise rotation angle when looking along the z-axis towards the origin in radians.</param>
        /// <returns>The fp4x4 rotation matrix that rotates around the z-axis.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 RotateZ(fp angle)
        {
            // {{c_2, -s_2, 0}, {s_2, c_2, 0}, {0, 0, 1}}
            fp s, c;
            sincos(angle, out s, out c);
            return fp4x4(c,    -s,   fp.zero, fp.zero,
                            s,    c,    fp.zero, fp.zero,
                            fp.zero, fp.zero, fp.one, fp.zero,
                            fp.zero, fp.zero, fp.zero, fp.one);

        }

        /// <summary>Returns a fp4x4 scale matrix given 3 axis scales.</summary>
        /// <param name="s">The uniform scaling factor.</param>
        /// <returns>The fp4x4 matrix that represents a uniform scale.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 Scale(fp s)
        {
            return fp4x4(s,    fp.zero, fp.zero, fp.zero,
                            fp.zero, s,    fp.zero, fp.zero,
                            fp.zero, fp.zero, s,    fp.zero,
                            fp.zero, fp.zero, fp.zero, fp.one);
        }

        /// <summary>Returns a fp4x4 scale matrix given a fp3 vector containing the 3 axis scales.</summary>
        /// <param name="x">The x-axis scaling factor.</param>
        /// <param name="y">The y-axis scaling factor.</param>
        /// <param name="z">The z-axis scaling factor.</param>
        /// <returns>The fp4x4 matrix that represents a non-uniform scale.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 Scale(fp x, fp y, fp z)
        {
            return fp4x4(x,    fp.zero, fp.zero, fp.zero,
                            fp.zero, y,    fp.zero, fp.zero,
                            fp.zero, fp.zero, z,    fp.zero,
                            fp.zero, fp.zero, fp.zero, fp.one);
        }

        /// <summary>Returns a fp4x4 scale matrix given a fp3 vector containing the 3 axis scales.</summary>
        /// <param name="scales">The vector containing scale factors for each axis.</param>
        /// <returns>The fp4x4 matrix that represents a non-uniform scale.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 Scale(fp3 scales)
        {
            return Scale(scales.x, scales.y, scales.z);
        }

        /// <summary>Returns a fp4x4 translation matrix given a fp3 translation vector.</summary>
        /// <param name="vector">The translation vector.</param>
        /// <returns>The fp4x4 translation matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 Translate(fp3 vector)
        {
            return fp4x4(fp4(fp.one, fp.zero, fp.zero, fp.zero),
                            fp4(fp.zero, fp.one, fp.zero, fp.zero),
                            fp4(fp.zero, fp.zero, fp.one, fp.zero),
                            fp4(vector.x, vector.y, vector.z, fp.one));
        }

        /// <summary>
        /// Returns a fp4x4 view matrix given an eye position, a target point and a unit length up vector.
        /// The up vector is assumed to be unit length, the eye and target points are assumed to be distinct and
        /// the vector between them is assumes to be collinear with the up vector.
        /// If these assumptions are not met use fp4x4.LookRotationSafe instead.
        /// </summary>
        /// <param name="eye">The eye position.</param>
        /// <param name="target">The view target position.</param>
        /// <param name="up">The eye up direction.</param>
        /// <returns>The fp4x4 view matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 LookAt(fp3 eye, fp3 target, fp3 up)
        {
            fp3x3 rot = fp3x3.LookRotation(normalize(target - eye), up);

            fp4x4 matrix;
            matrix.c0 = fp4(rot.c0, fp.zero);
            matrix.c1 = fp4(rot.c1, fp.zero);
            matrix.c2 = fp4(rot.c2, fp.zero);
            matrix.c3 = fp4(eye, fp.one);
            return matrix;
        }

        /// <summary>
        /// Returns a fp4x4 centered orthographic projection matrix.
        /// </summary>
        /// <param name="width">The width of the view volume.</param>
        /// <param name="height">The height of the view volume.</param>
        /// <param name="near">The distance to the near plane.</param>
        /// <param name="far">The distance to the far plane.</param>
        /// <returns>The fp4x4 centered orthographic projection matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 Ortho(fp width, fp height, fp near, fp far)
        {
            fp rcpdx = fp.one / width;
            fp rcpdy = fp.one / height;
            fp rcpdz = fp.one / (far - near);

            return fp4x4(
                (fp)2.0f * rcpdx, fp.zero, fp.zero, fp.zero,
                fp.zero, (fp)2.0f * rcpdy, fp.zero, fp.zero,
                fp.zero, fp.zero, (fp)(-2.0f) * rcpdz, -(far + near) * rcpdz,
                fp.zero, fp.zero, fp.zero, fp.one
            );
        }

        /// <summary>
        /// Returns a fp4x4 off-center orthographic projection matrix.
        /// </summary>
        /// <param name="left">The minimum x-coordinate of the view volume.</param>
        /// <param name="right">The maximum x-coordinate of the view volume.</param>
        /// <param name="bottom">The minimum y-coordinate of the view volume.</param>
        /// <param name="top">The minimum y-coordinate of the view volume.</param>
        /// <param name="near">The distance to the near plane.</param>
        /// <param name="far">The distance to the far plane.</param>
        /// <returns>The fp4x4 off-center orthographic projection matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 OrthoOffCenter(fp left, fp right, fp bottom, fp top, fp near, fp far)
        {
            fp rcpdx = fp.one / (right - left);
            fp rcpdy = fp.one / (top - bottom);
            fp rcpdz = fp.one / (far - near);

            return fp4x4(
                (fp)2.0f * rcpdx, fp.zero, fp.zero, -(right + left) * rcpdx,
                fp.zero, (fp)2.0f * rcpdy, fp.zero, -(top + bottom) * rcpdy,
                fp.zero, fp.zero, (fp)(-2.0f) * rcpdz, -(far + near) * rcpdz,
                fp.zero, fp.zero, fp.zero, fp.one
            );
        }

        /// <summary>
        /// Returns a fp4x4 perspective projection matrix based on field of view.
        /// </summary>
        /// <param name="verticalFov">Vertical Field of view in radians.</param>
        /// <param name="aspect">X:Y aspect ratio.</param>
        /// <param name="near">Distance to near plane. Must be greater than zero.</param>
        /// <param name="far">Distance to far plane. Must be greater than zero.</param>
        /// <returns>The fp4x4 perspective projection matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 PerspectiveFov(fp verticalFov, fp aspect, fp near, fp far)
        {
            fp cotangent = fp.one / fp.Tan(verticalFov * fp.half);
            fp rcpdz = fp.one / (near - far);

            return fp4x4(
                cotangent / aspect, fp.zero,       fp.zero,                   fp.zero,
                fp.zero,               cotangent,  fp.zero,                   fp.zero,
                fp.zero,               fp.zero,       (far + near) * rcpdz,   (fp)2.0f * near * far * rcpdz,
                fp.zero,               fp.zero,      -fp.one,                   fp.zero
                );
        }

        /// <summary>
        /// Returns a fp4x4 off-center perspective projection matrix.
        /// </summary>
        /// <param name="left">The x-coordinate of the left side of the clipping frustum at the near plane.</param>
        /// <param name="right">The x-coordinate of the right side of the clipping frustum at the near plane.</param>
        /// <param name="bottom">The y-coordinate of the bottom side of the clipping frustum at the near plane.</param>
        /// <param name="top">The y-coordinate of the top side of the clipping frustum at the near plane.</param>
        /// <param name="near">Distance to the near plane. Must be greater than zero.</param>
        /// <param name="far">Distance to the far plane. Must be greater than zero.</param>
        /// <returns>The fp4x4 off-center perspective projection matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 PerspectiveOffCenter(fp left, fp right, fp bottom, fp top, fp near, fp far)
        {
            fp rcpdz = fp.one / (near - far);
            fp rcpWidth = fp.one / (right - left);
            fp rcpHeight = fp.one / (top - bottom);

            return fp4x4(
                (fp)2.0f * near * rcpWidth,     fp.zero,                       (left + right) * rcpWidth,     fp.zero,
                fp.zero,                       (fp)2.0f * near * rcpHeight,    (bottom + top) * rcpHeight,    fp.zero,
                fp.zero,                       fp.zero,                        (far + near) * rcpdz,          (fp)2.0f * near * far * rcpdz,
                fp.zero,                       fp.zero,                       -fp.one,                          fp.zero
                );
        }

        /// <summary>
        /// Returns a fp4x4 matrix representing a combined scale-, rotation- and translation transform.
        /// Equivalent to mul(translationTransform, mul(rotationTransform, scaleTransform)).
        /// </summary>
        /// <param name="translation">The translation vector.</param>
        /// <param name="rotation">The quaternion rotation.</param>
        /// <param name="scale">The scaling factors of each axis.</param>
        /// <returns>The fp4x4 matrix representing the translation, rotation, and scale by the inputs.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 TRS(fp3 translation, fpquaternion rotation, fp3 scale)
        {
            fp3x3 r = fp3x3(rotation);
            return fp4x4(  fp4(r.c0 * scale.x, fp.zero),
                              fp4(r.c1 * scale.y, fp.zero),
                              fp4(r.c2 * scale.z, fp.zero),
                              fp4(translation, fp.one));
        }
    }

    partial class fpmath
    {
        /// <summary>
        /// Extracts a fp3x3 from the upper left 3x3 of a fp4x4.
        /// </summary>
        /// <param name="f4x4"><see cref="fp4x4"/> to extract a fp3x3 from.</param>
        /// <returns>Upper left 3x3 matrix as fp3x3.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 fp3x3(fp4x4 f4x4)
        {
            return new fp3x3(f4x4);
        }

        /// <summary>Returns a fp3x3 matrix constructed from a quaternion.</summary>
        /// <param name="rotation">The quaternion representing a rotation.</param>
        /// <returns>The fp3x3 constructed from a quaternion.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 fp3x3(fpquaternion rotation)
        {
            return new fp3x3(rotation);
        }

        /// <summary>Returns a fp4x4 constructed from a fp3x3 rotation matrix and a fp3 translation vector.</summary>
        /// <param name="rotation">The fp3x3 rotation matrix.</param>
        /// <param name="translation">The translation vector.</param>
        /// <returns>The fp4x4 constructed from a rotation and translation.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 fp4x4(fp3x3 rotation, fp3 translation)
        {
            return new fp4x4(rotation, translation);
        }

        /// <summary>Returns a fp4x4 constructed from a quaternion and a fp3 translation vector.</summary>
        /// <param name="rotation">The quaternion rotation.</param>
        /// <param name="translation">The translation vector.</param>
        /// <returns>The fp4x4 constructed from a rotation and translation.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 fp4x4(fpquaternion rotation, fp3 translation)
        {
            return new fp4x4(rotation, translation);
        }

        /// <summary>Returns a fp4x4 constructed from a RigidTransform.</summary>
        /// <param name="transform">The rigid transformation.</param>
        /// <returns>The fp4x4 constructed from a RigidTransform.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp4x4 fp4x4(FpRigidTransform transform)
        {
            return new fp4x4(transform);
        }

        /// <summary>Returns an orthonormalized version of a fp3x3 matrix.</summary>
        /// <param name="i">The fp3x3 to be orthonormalized.</param>
        /// <returns>The orthonormalized fp3x3 matrix.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static fp3x3 orthonormalize(fp3x3 i)
        {
            fp3x3 o;

            fp3 u = i.c0;
            fp3 v = i.c1 - i.c0 * fpmath.dot(i.c1, i.c0);

            fp lenU = fpmath.length(u);
            fp lenV = fpmath.length(v);

            const uint smallValue = 0x0da24260;
            fp epsilon = fp.FromRaw(smallValue);

            bool c = lenU > epsilon && lenV > epsilon;

            o.c0 = fpmath.select(fp3(fp.one, fp.zero, fp.zero), u / lenU, c);
            o.c1 = fpmath.select(fp3(fp.zero, fp.one, fp.zero), v / lenV, c);
            o.c2 = fpmath.cross(o.c0, o.c1);

            return o;
        }
    }
}
