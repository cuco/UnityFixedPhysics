using Unity.Collections;
using Unity.Mathematics.FixedPoint;
using UnityEngine.Assertions;

namespace Fixed.Physics
{
    public struct SurfaceConstraintInfo // TODO: Move into SimplexSolver
    {
        // Info of interest for the character
        public Plane Plane;
        public fp3 Velocity;

        // Hit body info
        public int RigidBodyIndex;
        public ColliderKey ColliderKey;
        public fp3 HitPosition;

        // Internal state
        public int Priority;
        public bool Touched;
        public bool IsTooSteep;
        public bool IsMaxSlope;
    }

    public static class SimplexSolver
    {
        static readonly fp k_Epsilon = fp.FromRaw(0x38d1b717);

        public static unsafe void Solve(
            fp deltaTime, fp minDeltaTime, fp3 up, fp maxVelocity,
            NativeList<SurfaceConstraintInfo> constraints, ref fp3 position, ref fp3 velocity, out fp integratedTime, bool useConstraintVelocities = true
        )
        {
            // List of planes to solve against (up to 4)
            SurfaceConstraintInfo* supportPlanes = stackalloc SurfaceConstraintInfo[4];
            int numSupportPlanes = 0;

            fp remainingTime = deltaTime;
            fp currentTime = fp.zero;

            // Clamp the input velocity to max movement speed
            Math.ClampToMaxLength(maxVelocity, ref velocity);

            while (remainingTime > fp.zero)
            {
                int hitIndex = -1;
                fp minCollisionTime = remainingTime;

                // Iterate over constraints and solve them
                for (int i = 0; i < constraints.Length; i++)
                {
                    if (constraints[i].Touched) continue;

                    SurfaceConstraintInfo constraint = constraints[i];

                    fp3 relVel = velocity - (useConstraintVelocities ? constraint.Velocity : fp3.zero);
                    fp relProjVel = -fpmath.dot(relVel, constraint.Plane.Normal);
                    if (relProjVel < k_Epsilon)
                    {
                        continue;
                    }

                    // Clamp distance to 0, since penetration is handled by constraint.Velocity already
                    fp distance = fpmath.max(constraint.Plane.Distance, fp.zero);
                    if (distance < minCollisionTime * relProjVel)
                    {
                        minCollisionTime = distance / relProjVel;
                        hitIndex = i;
                    }
                }

                // Integrate
                {
                    currentTime += minCollisionTime;
                    remainingTime -= minCollisionTime;
                    position += minCollisionTime * velocity;
                }

                if (hitIndex < 0 || currentTime > minDeltaTime)
                {
                    break;
                }

                // Mark constraint as touched
                {
                    var constraint = constraints[hitIndex];
                    constraint.Touched = true;
                    constraints[hitIndex] = constraint;
                }

                // Add the hit to the current list of active planes
                supportPlanes[numSupportPlanes] = constraints[hitIndex];
                if (!useConstraintVelocities)
                {
                    supportPlanes[numSupportPlanes].Velocity = fp3.zero;
                }
                numSupportPlanes++;

                // Solve support planes
                ExamineActivePlanes(up, supportPlanes, ref numSupportPlanes, ref velocity);

                // Clamp the solved velocity to max movement speed
                Math.ClampToMaxLength(maxVelocity, ref velocity);

                // Can't handle more than 4 support planes
                if (numSupportPlanes == 4)
                {
                    break;
                }
            }

            integratedTime = currentTime;
        }

        public static unsafe void ExamineActivePlanes(fp3 up, SurfaceConstraintInfo* supportPlanes, ref int numSupportPlanes, ref fp3 velocity)
        {
            switch (numSupportPlanes)
            {
                case 1:
                {
                    Solve1d(supportPlanes[0], ref velocity);
                    return;
                }
                case 2:
                {
                    // Test whether we need plane 0 at all
                    fp3 tempVelocity = velocity;
                    Solve1d(supportPlanes[1], ref tempVelocity);

                    bool plane0Used = Test1d(supportPlanes[0], tempVelocity);
                    if (!plane0Used)
                    {
                        // Compact the buffer and reduce size
                        supportPlanes[0] = supportPlanes[1];
                        numSupportPlanes = 1;

                        // Write back the result
                        velocity = tempVelocity;
                    }
                    else
                    {
                        Solve2d(up, supportPlanes[0], supportPlanes[1], ref velocity);
                    }

                    return;
                }
                case 3:
                {
                    // Try to drop both planes
                    fp3 tempVelocity = velocity;
                    Solve1d(supportPlanes[2], ref tempVelocity);

                    bool plane0Used = Test1d(supportPlanes[0], tempVelocity);
                    if (!plane0Used)
                    {
                        bool plane1Used = Test1d(supportPlanes[1], tempVelocity);
                        if (!plane1Used)
                        {
                            // Compact the buffer and reduce size
                            supportPlanes[0] = supportPlanes[2];
                            numSupportPlanes = 1;
                            goto case 1;
                        }
                    }

                    // Try to drop plane 0 or 1
                    for (int testPlane = 0; testPlane < 2; testPlane++)
                    {
                        tempVelocity = velocity;
                        Solve2d(up, supportPlanes[testPlane], supportPlanes[2], ref tempVelocity);

                        bool planeUsed = Test1d(supportPlanes[1 - testPlane], tempVelocity);
                        if (!planeUsed)
                        {
                            supportPlanes[0] = supportPlanes[testPlane];
                            supportPlanes[1] = supportPlanes[2];
                            numSupportPlanes--;
                            goto case 2;
                        }
                    }

                    // Try solve all three
                    Solve3d(up, supportPlanes[0], supportPlanes[1], supportPlanes[2], ref velocity);

                    return;
                }
                case 4:
                {
                    for (int i = 0; i < 3; i++)
                    {
                        fp3 tempVelocity = velocity;
                        Solve3d(up, supportPlanes[(i + 1) % 3], supportPlanes[(i + 2) % 3], supportPlanes[3], ref tempVelocity);
                        bool planeUsed = Test1d(supportPlanes[i], tempVelocity);
                        if (!planeUsed)
                        {
                            supportPlanes[i] = supportPlanes[2];
                            supportPlanes[2] = supportPlanes[3];
                            numSupportPlanes = 3;
                            goto case 3;
                        }
                    }

                    // Nothing can be dropped so we've failed to solve,
                    // now we do all 3d combinations
                    fp3 tempVel = velocity;
                    SurfaceConstraintInfo sp0 = supportPlanes[0];
                    SurfaceConstraintInfo sp1 = supportPlanes[1];
                    SurfaceConstraintInfo sp2 = supportPlanes[2];
                    SurfaceConstraintInfo sp3 = supportPlanes[3];
                    Solve3d(up, sp0, sp1, sp2, ref tempVel);
                    Solve3d(up, sp0, sp1, sp3, ref tempVel);
                    Solve3d(up, sp0, sp2, sp3, ref tempVel);
                    Solve3d(up, sp1, sp2, sp3, ref tempVel);

                    velocity = tempVel;

                    return;
                }
                default:
                {
                    // Can't have more than 4 and less than 1 plane
                    Assert.IsTrue(false);
                    break;
                }
            }
        }

        public static void Solve1d(SurfaceConstraintInfo constraint, ref fp3 velocity)
        {
            fp3 groundVelocity = constraint.Velocity;
            fp3 relVel = velocity - groundVelocity;
            fp planeVel = fpmath.dot(relVel, constraint.Plane.Normal);
            relVel -= planeVel * constraint.Plane.Normal;

            velocity = relVel + groundVelocity;
        }

        public static bool Test1d(SurfaceConstraintInfo constraint, fp3 velocity)
        {
            fp3 relVel = velocity - constraint.Velocity;
            fp planeVel = fpmath.dot(relVel, constraint.Plane.Normal);
            return planeVel < -k_Epsilon;
        }

        public static void Solve2d(fp3 up, SurfaceConstraintInfo constraint0, SurfaceConstraintInfo constraint1, ref fp3 velocity)
        {
            fp3 plane0 = constraint0.Plane.Normal;
            fp3 plane1 = constraint1.Plane.Normal;

            // Calculate the free axis
            fp3 axis = fpmath.cross(plane0, plane1);
            fp axisLen2 = fpmath.lengthsq(axis);

            // Check for parallel planes
            if (axisLen2 < k_Epsilon)
            {
                // Do the planes sequentially
                Sort2d(ref constraint0, ref constraint1);
                Solve1d(constraint1, ref velocity);
                Solve1d(constraint0, ref velocity);

                return;
            }

            fp invAxisLen = fpmath.rsqrt(axisLen2);
            axis *= invAxisLen;

            // Calculate the velocity of the free axis
            fp3 axisVel;
            {
                fp4x4 m = new fp4x4();
                fp3 r0 = fpmath.cross(plane0, plane1);
                fp3 r1 = fpmath.cross(plane1, axis);
                fp3 r2 = fpmath.cross(axis, plane0);
                m.c0 = new fp4(r0, fp.zero);
                m.c1 = new fp4(r1, fp.zero);
                m.c2 = new fp4(r2, fp.zero);
                m.c3 = new fp4(fp.zero, fp.zero, fp.zero, fp.one);

                fp3 sVel = constraint0.Velocity + constraint1.Velocity;
                fp3 t = new fp3(
                    fpmath.dot(axis, sVel) * fp.half,
                    fpmath.dot(plane0, constraint0.Velocity),
                    fpmath.dot(plane1, constraint1.Velocity));

                axisVel = fpmath.rotate(m, t);
                axisVel *= invAxisLen;
            }

            fp3 groundVelocity = axisVel;
            fp3 relVel = velocity - groundVelocity;

            fp vel2 = fpmath.lengthsq(relVel);
            fp axisVert = fpmath.dot(up, axis);
            fp axisProjVel = fpmath.dot(relVel, axis);

            velocity = groundVelocity + axis * axisProjVel;
        }

        public static void Solve3d(fp3 up, SurfaceConstraintInfo constraint0, SurfaceConstraintInfo constraint1, SurfaceConstraintInfo constraint2, ref fp3 velocity)
        {
            fp3 plane0 = constraint0.Plane.Normal;
            fp3 plane1 = constraint1.Plane.Normal;
            fp3 plane2 = constraint2.Plane.Normal;

            fp4x4 m = new fp4x4();
            fp3 r0 = fpmath.cross(plane1, plane2);
            fp3 r1 = fpmath.cross(plane2, plane0);
            fp3 r2 = fpmath.cross(plane0, plane1);
            m.c0 = new fp4(r0, fp.zero);
            m.c1 = new fp4(r1, fp.zero);
            m.c2 = new fp4(r2, fp.zero);
            m.c3 = new fp4(fp.zero, fp.zero, fp.zero, fp.one);

            fp det = fpmath.dot(r0, plane0);
            fp tst = fpmath.abs(det);
            if (tst < k_Epsilon)
            {
                Sort3d(ref constraint0, ref constraint1, ref constraint2);
                Solve2d(up, constraint1, constraint2, ref velocity);
                Solve2d(up, constraint0, constraint2, ref velocity);
                Solve2d(up, constraint0, constraint1, ref velocity);

                return;
            }

            fp3 t = new fp3(
                fpmath.dot(plane0, constraint0.Velocity),
                fpmath.dot(plane1, constraint1.Velocity),
                fpmath.dot(plane2, constraint2.Velocity));

            fp3 pointVel = fpmath.rotate(m, t);
            pointVel /= det;

            velocity = pointVel;
        }

        public static void Sort2d(ref SurfaceConstraintInfo plane0, ref SurfaceConstraintInfo plane1)
        {
            int priority0 = plane0.Priority;
            int priority1 = plane1.Priority;
            if (priority0 > priority1)
            {
                SwapPlanes(ref plane0, ref plane1);
            }
        }

        public static void Sort3d(ref SurfaceConstraintInfo plane0, ref SurfaceConstraintInfo plane1, ref SurfaceConstraintInfo plane2)
        {
            int priority0 = plane0.Priority;
            int priority1 = plane1.Priority;
            int priority2 = plane2.Priority;
            if (priority0 <= priority1)
            {
                if (priority1 <= priority2)
                {
                    // 0, 1, 2
                }
                else if (priority0 <= priority2)
                {
                    // 0, 2, 1
                    SwapPlanes(ref plane1, ref plane2);
                }
                else
                {
                    // 1, 2, 0
                    SwapPlanes(ref plane0, ref plane1);
                    SwapPlanes(ref plane0, ref plane2);
                }
            }
            else
            {
                if (priority2 < priority1)
                {
                    // 2, 1, 0
                    SwapPlanes(ref plane0, ref plane2);
                }
                else if (priority2 > priority0)
                {
                    // 1, 0, 2
                    SwapPlanes(ref plane0, ref plane1);
                }
                else
                {
                    // 2, 0, 1
                    SwapPlanes(ref plane0, ref plane1);
                    SwapPlanes(ref plane1, ref plane2);
                }
            }
        }

        public static void SwapPlanes(ref SurfaceConstraintInfo plane0, ref SurfaceConstraintInfo plane1)
        {
            var temp = plane0;
            plane0 = plane1;
            plane1 = temp;
        }
    }
}
