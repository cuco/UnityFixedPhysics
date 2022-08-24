using Unity.Collections;
using NUnit.Framework;
using Unity.Mathematics.FixedPoint;
using Random = Unity.Mathematics.FixedPoint.Random;
using static Fixed.Physics.Math;

namespace Fixed.Physics.Tests.Joints
{
    /// <summary>
    /// These tests generate random motions and joints, simulate them for several steps, then verifies that the joint error is nearly zero.
    /// Passing test does not necessarily mean good-looking joint behavior or stability for systems of multiple constraints, but the test
    /// will catch a lot of basic mathematical errors in the solver.
    /// </summary>
    class JointTests
    {
        //
        // Tiny simulation for a single body pair and joint, used by all of the tests
        //

        void applyGravity(ref MotionVelocity velocity, ref MotionData motion, fp3 gravity, fp timestep)
        {
            if (velocity.InverseMass > (fp)0.0f)
            {
                velocity.LinearVelocity += gravity * timestep;
            }
        }

        static void integrate(ref MotionVelocity velocity, ref MotionData motion, fp timestep)
        {
            Integrator.Integrate(ref motion.WorldFromMotion, velocity, timestep);
        }

        //
        // Random test data generation
        //

        static fp3 generateRandomCardinalAxis(ref Random rnd)
        {
            fp3 axis = fp3.zero;
            axis[rnd.NextInt(3)] = rnd.NextBool() ? (fp)1 : -(fp)1;
            return axis;
        }

        static FpRigidTransform generateRandomTransform(ref Random rnd)
        {
            // Random rotation: 1 in 4 are identity, 3 in 16 are 90 or 180 degrees about i j or k, the rest are uniform random
            fpquaternion rot = fpquaternion.identity;
            if (rnd.NextInt(4) > 0)
            {
                if (rnd.NextInt(4) > 0)
                {
                    rot = rnd.NextQuaternionRotation();
                }
                else
                {
                    fp angle = rnd.NextBool() ? (fp)90 : (fp)180;
                    rot = fpquaternion.AxisAngle(generateRandomCardinalAxis(ref rnd), angle);
                }
            }

            return new FpRigidTransform()
            {
                pos = rnd.NextInt(4) == 0 ? fp3.zero : rnd.Nextfp3(-(fp)1.0f, (fp)1.0f),
                rot = rot
            };
        }

        void generateRandomMotion(ref Random rnd, out MotionVelocity velocity, out MotionData motion, bool allowInfiniteMass)
        {
            motion = new MotionData
            {
                WorldFromMotion = generateRandomTransform(ref rnd),
                BodyFromMotion = generateRandomTransform(ref rnd)
            };

            fp3 inertia = rnd.Nextfp3((fp)1e-3f, (fp)100.0f);
            switch (rnd.NextInt(3))
            {
                case 0: // all values random
                    break;
                case 1: // two values the same
                    int index = rnd.NextInt(3);
                    inertia[(index + 1) % 2] = inertia[index];
                    break;
                case 2: // all values the same
                    inertia = inertia.zzz;
                    break;
            }

            fp3 nextLinVel;
            if (rnd.NextBool())
            {
                nextLinVel = fp3.zero;
            }
            else
            {
                nextLinVel = rnd.Nextfp3(-(fp)50.0f, (fp)50.0f);
            }
            fp3 nextAngVel;
            if (rnd.NextBool())
            {
                nextAngVel = fp3.zero;
            }
            else
            {
                nextAngVel = rnd.Nextfp3(-(fp)50.0f, (fp)50.0f);
            }
            fp3 nextInertia;
            fp nextMass;
            if (allowInfiniteMass && rnd.NextBool())
            {
                nextInertia = fp3.zero;
                nextMass = (fp)0.0f;
            }
            else
            {
                nextMass = rnd.NextFloat((fp)1e-3f, (fp)100.0f);
                nextInertia = (fp)1.0f / inertia;
            }
            velocity = new MotionVelocity
            {
                LinearVelocity = nextLinVel,
                AngularVelocity = nextAngVel,
                InverseInertia = nextInertia,
                InverseMass = nextMass
            };
        }

        //
        // Helpers
        //

        static FpRigidTransform getWorldFromBody(MotionData motion)
        {
            return fpmath.mul(motion.WorldFromMotion, fpmath.inverse(motion.BodyFromMotion));
        }

        static fp3 getBodyPointVelocity(MotionVelocity velocity, MotionData motion, fp3 positionInBodySpace, out fp angularLength)
        {
            fp3 positionInMotion = fpmath.transform(fpmath.inverse(motion.BodyFromMotion), positionInBodySpace);
            fp3 angularInMotion = fpmath.cross(velocity.AngularVelocity, positionInMotion);
            angularLength = fpmath.length(angularInMotion);
            fp3 angularInWorld = fpmath.rotate(motion.WorldFromMotion, angularInMotion);
            return angularInWorld + velocity.LinearVelocity;
        }

        //
        // Test runner
        //

        delegate Joint GenerateJoint(ref Random rnd);

        unsafe static void SolveSingleJoint(Joint jointData, int numIterations, fp timestep,
            ref MotionVelocity velocityA, ref MotionVelocity velocityB, ref MotionData motionA, ref MotionData motionB, out NativeStream jacobiansOut)
        {
            var stepInput = new Solver.StepInput
            {
                IsLastIteration = false,
                InvNumSolverIterations = (fp)1.0f / (fp)numIterations,
                Timestep = timestep,
                InvTimestep = timestep > (fp)0.0f ? (fp)1.0f / timestep : (fp)0.0f
            };

            // Build jacobians
            jacobiansOut = new NativeStream(1, Allocator.Temp);
            {
                NativeStream.Writer jacobianWriter = jacobiansOut.AsWriter();
                jacobianWriter.BeginForEachIndex(0);
                Solver.BuildJointJacobian(jointData, velocityA, velocityB, motionA, motionB, timestep, numIterations, ref jacobianWriter);
                jacobianWriter.EndForEachIndex();
            }

            var eventWriter = new NativeStream.Writer(); // no events expected

            // Solve the joint
            for (int iIteration = 0; iIteration < numIterations; iIteration++)
            {
                stepInput.IsLastIteration = (iIteration == numIterations - 1);
                NativeStream.Reader jacobianReader = jacobiansOut.AsReader();
                var jacIterator = new JacobianIterator(jacobianReader, 0);
                while (jacIterator.HasJacobiansLeft())
                {
                    ref JacobianHeader header = ref jacIterator.ReadJacobianHeader();
                    header.Solve(ref velocityA, ref velocityB, stepInput, ref eventWriter, ref eventWriter,
                        false, Solver.MotionStabilizationInput.Default, Solver.MotionStabilizationInput.Default);
                }
            }

            // After solving, integrate motions
            integrate(ref velocityA, ref motionA, timestep);
            integrate(ref velocityB, ref motionB, timestep);
        }

        unsafe void RunJointTest(string testName, GenerateJoint generateJoint)
        {
            uint numTests = 1000;
            uint dbgTest = 2472156941;
            if (dbgTest > 0)
            {
                numTests = 1;
            }

            Random rnd = new Random(58297436);
            for (int iTest = 0; iTest < numTests; iTest++)
            {
                if (dbgTest > 0)
                {
                    rnd.state = dbgTest;
                }
                uint state = rnd.state;

                // Generate a random ball and socket joint
                Joint jointData = generateJoint(ref rnd);

                // Generate random motions
                MotionVelocity velocityA, velocityB;
                MotionData motionA, motionB;
                generateRandomMotion(ref rnd, out velocityA, out motionA, true);
                generateRandomMotion(ref rnd, out velocityB, out motionB, !velocityA.IsKinematic);

                // Simulate the joint
                {
                    // Build input
                    fp timestep = (fp)1.0f / (fp)50.0f;
                    const int numIterations = 4;
                    const int numSteps = 15;
                    fp3 gravity = new fp3((fp)0.0f, -(fp)9.81f, (fp)0.0f);

                    // Simulate
                    for (int iStep = 0; iStep < numSteps; iStep++)
                    {
                        // Before solving, apply gravity
                        applyGravity(ref velocityA, ref motionA, gravity, timestep);
                        applyGravity(ref velocityB, ref motionB, gravity, timestep);

                        // Solve and integrate
                        SolveSingleJoint(jointData, numIterations, timestep, ref velocityA, ref velocityB, ref motionA, ref motionB, out NativeStream jacobians);

                        // Last step, check the joint error
                        if (iStep == numSteps - 1)
                        {
                            NativeStream.Reader jacobianReader = jacobians.AsReader();
                            var jacIterator = new JacobianIterator(jacobianReader, 0);
                            string failureMessage = testName + " failed " + iTest + " (" + state + ")";
                            while (jacIterator.HasJacobiansLeft())
                            {
                                ref JacobianHeader header = ref jacIterator.ReadJacobianHeader();
                                switch (header.Type)
                                {
                                    case JacobianType.LinearLimit:
                                        Assert.Less(header.AccessBaseJacobian<LinearLimitJacobian>().InitialError, 1e-3f, failureMessage + ": LinearLimitJacobian");
                                        break;
                                    case JacobianType.AngularLimit1D:
                                        Assert.Less(header.AccessBaseJacobian<AngularLimit1DJacobian>().InitialError, 1e-2f, failureMessage + ": AngularLimit1DJacobian");
                                        break;
                                    case JacobianType.AngularLimit2D:
                                        Assert.Less(header.AccessBaseJacobian<AngularLimit2DJacobian>().InitialError, 1e-2f, failureMessage + ": AngularLimit2DJacobian");
                                        break;
                                    case JacobianType.AngularLimit3D:
                                        Assert.Less(header.AccessBaseJacobian<AngularLimit3DJacobian>().InitialError, 1e-2f, failureMessage + ": AngularLimit3DJacobian");
                                        break;
                                    default:
                                        Assert.Fail(failureMessage + ": unexpected jacobian type");
                                        break;
                                }
                            }
                        }

                        // Cleanup
                        jacobians.Dispose();
                    }
                }
            }
        }

        //
        // Tests
        //

        static void generateRandomPivots(ref Random rnd, out fp3 pivotA, out fp3 pivotB)
        {
            pivotA = rnd.NextBool() ? fp3.zero : rnd.Nextfp3(-(fp)1.0f, (fp)1.0f);
            pivotB = rnd.NextBool() ? fp3.zero : rnd.Nextfp3(-(fp)1.0f, (fp)1.0f);
        }

        static void generateRandomAxes(ref Random rnd, out fp3 axisA, out fp3 axisB)
        {
            axisA = rnd.NextInt(4) == 0 ? generateRandomCardinalAxis(ref rnd) : rnd.Nextfp3Direction();
            axisB = rnd.NextInt(4) == 0 ? generateRandomCardinalAxis(ref rnd) : rnd.Nextfp3Direction();
        }

        static void generateRandomLimits(ref Random rnd, fp minClosed, fp maxClosed, out fp min, out fp max)
        {
            min = rnd.NextBool() ? fp.min_value : rnd.NextFloat(minClosed, maxClosed);
            max = rnd.NextBool() ? rnd.NextBool() ? fp.max_value : min : rnd.NextFloat(min, maxClosed);
        }

        Joint CreateTestJoint(PhysicsJoint joint) => new Joint
        {
            AFromJoint = joint.BodyAFromJoint.AsMTransform(),
            BFromJoint = joint.BodyBFromJoint.AsMTransform(),
            Constraints = joint.GetConstraints()
        };

        [Test]
        public unsafe void BallAndSocketTest()
        {
            RunJointTest("BallAndSocketTest", (ref Random rnd) =>
            {
                generateRandomPivots(ref rnd, out fp3 pivotA, out fp3 pivotB);
                return CreateTestJoint(PhysicsJoint.CreateBallAndSocket(pivotA, pivotB));
            });
        }

        [Test]
        public unsafe void StiffSpringTest()
        {
            RunJointTest("StiffSpringTest", (ref Random rnd) =>
            {
                generateRandomPivots(ref rnd, out fp3 pivotA, out fp3 pivotB);
                generateRandomLimits(ref rnd, (fp)0.0f, fp.half, out fp minDistance, out fp maxDistance);
                return CreateTestJoint(PhysicsJoint.CreateLimitedDistance(pivotA, pivotB, new FloatRange(minDistance, maxDistance)));
            });
        }

        [Test]
        public unsafe void PrismaticTest()
        {
            RunJointTest("PrismaticTest", (ref Random rnd) =>
            {
                var jointFrameA = new BodyFrame();
                var jointFrameB = new BodyFrame();
                generateRandomPivots(ref rnd, out jointFrameA.Position, out jointFrameB.Position);
                generateRandomAxes(ref rnd, out jointFrameA.Axis, out jointFrameB.Axis);
                Math.CalculatePerpendicularNormalized(jointFrameA.Axis, out jointFrameA.PerpendicularAxis, out _);
                Math.CalculatePerpendicularNormalized(jointFrameB.Axis, out jointFrameB.PerpendicularAxis, out _);
                var distance = new FloatRange { Min = rnd.NextFloat(-fp.half, fp.half) };
                distance.Max = rnd.NextBool() ? distance.Min : rnd.NextFloat(distance.Min, fp.half); // note, can't use open limits because the accuracy can get too low as the pivots separate
                return CreateTestJoint(PhysicsJoint.CreatePrismatic(jointFrameA, jointFrameB, distance));
            });
        }

        [Test]
        public unsafe void HingeTest()
        {
            RunJointTest("HingeTest", (ref Random rnd) =>
            {
                var jointFrameA = new BodyFrame();
                var jointFrameB = new BodyFrame();
                generateRandomPivots(ref rnd, out jointFrameA.Position, out jointFrameB.Position);
                generateRandomAxes(ref rnd, out jointFrameA.Axis, out jointFrameB.Axis);
                Math.CalculatePerpendicularNormalized(jointFrameA.Axis, out jointFrameA.PerpendicularAxis, out _);
                Math.CalculatePerpendicularNormalized(jointFrameB.Axis, out jointFrameB.PerpendicularAxis, out _);
                return CreateTestJoint(PhysicsJoint.CreateHinge(jointFrameA, jointFrameB));
            });
        }

        [Test]
        public unsafe void LimitedHingeTest()
        {
            RunJointTest("LimitedHingeTest", (ref Random rnd) =>
            {
                var jointFrameA = new BodyFrame();
                var jointFrameB = new BodyFrame();
                generateRandomPivots(ref rnd, out jointFrameA.Position, out jointFrameB.Position);
                generateRandomAxes(ref rnd, out jointFrameA.Axis, out jointFrameB.Axis);
                Math.CalculatePerpendicularNormalized(jointFrameA.Axis, out jointFrameA.PerpendicularAxis, out _);
                Math.CalculatePerpendicularNormalized(jointFrameB.Axis, out jointFrameB.PerpendicularAxis, out _);
                FloatRange limits;
                generateRandomLimits(ref rnd, -(fp)fpmath.PI, (fp)fpmath.PI, out limits.Min, out limits.Max);
                return CreateTestJoint(PhysicsJoint.CreateLimitedHinge(jointFrameA, jointFrameB, limits));
            });
        }

        // TODO - test CreateRagdoll(), if it stays.  Doesn't fit nicely because it produces two JointDatas.

        [Test]
        public unsafe void FixedTest()
        {
            RunJointTest("FixedTest", (ref Random rnd) =>
            {
                var jointFrameA = new FpRigidTransform();
                var jointFrameB = new FpRigidTransform();
                generateRandomPivots(ref rnd, out jointFrameA.pos, out jointFrameB.pos);
                jointFrameA.rot = generateRandomTransform(ref rnd).rot;
                jointFrameB.rot = generateRandomTransform(ref rnd).rot;
                return CreateTestJoint(PhysicsJoint.CreateFixed(jointFrameA, jointFrameB));
            });
        }

        [Test]
        public unsafe void LimitedDOFTest()
        {
            RunJointTest("LimitedDOFTest", (ref Random rnd) =>
            {
                var linearAxes = new bool3(false);
                var angularAxes = new bool3(false);
                for (int i = 0; i < 3; i++) linearAxes[rnd.NextInt(0, 2)] = !linearAxes[rnd.NextInt(0, 2)];
                for (int i = 0; i < 3; i++) angularAxes[rnd.NextInt(0, 2)] = !angularAxes[rnd.NextInt(0, 2)];
                return CreateTestJoint(PhysicsJoint.CreateLimitedDOF(generateRandomTransform(ref rnd), linearAxes, angularAxes));
            });
        }

        [Test]
        public unsafe void TwistTest()
        {
            // Check that the twist constraint works in each axis.
            // Set up a constraint between a fixed and dynamic body, give the dynamic body
            // angular velocity about the limited axis, and verify that it stops at the limit
            for (int i = 0; i < 3; i++) // For each axis
            {
                for (int j = 0; j < 2; j++) // Negative / positive limit
                {
                    fp3 axis = fp3.zero;
                    axis[i] = (fp)1.0f;

                    MotionVelocity velocityA = new MotionVelocity
                    {
                        LinearVelocity = fp3.zero,
                        AngularVelocity = (j + j - 1) * axis,
                        InverseInertia = new fp3(1),
                        InverseMass = (fp)1
                    };

                    MotionVelocity velocityB = new MotionVelocity
                    {
                        LinearVelocity = fp3.zero,
                        AngularVelocity = fp3.zero,
                        InverseInertia = fp3.zero,
                        InverseMass = (fp)0.0f
                    };

                    MotionData motionA = new MotionData
                    {
                        WorldFromMotion = FpRigidTransform.identity,
                        BodyFromMotion = FpRigidTransform.identity
                    };

                    MotionData motionB = motionA;

                    fp angle = fp.half;
                    fp minLimit = (fp)(j - 1) * angle;
                    fp maxLimit = (fp)j * angle;

                    var jointData = new Joint
                    {
                        AFromJoint = MTransform.Identity,
                        BFromJoint = MTransform.Identity
                    };
                    jointData.Constraints.Add(Constraint.Twist(i, new FloatRange(minLimit, maxLimit), Constraint.DefaultSpringFrequency, Constraint.DefaultSpringDamping));
                    SolveSingleJoint(jointData, 4, (fp)1.0f, ref velocityA, ref velocityB, ref motionA, ref motionB, out NativeStream jacobians);

                    fpquaternion expectedOrientation = fpquaternion.AxisAngle(axis, minLimit + maxLimit);
                    Utils.TestUtils.AreEqual(expectedOrientation, motionA.WorldFromMotion.rot, (fp)1e-3f);
                    jacobians.Dispose();
                }
            }
        }

        [Test]
        public unsafe void ZeroDimensionTest()
        {
            RunJointTest("LimitedDOFTestZeroDimension", (ref Random rnd) =>
            {
                // Create a joint with 2 constraints that have 0 dimensions
                var noAxes = new bool3(false);
                return CreateTestJoint(PhysicsJoint.CreateLimitedDOF(generateRandomTransform(ref rnd), noAxes, noAxes));
            });
        }
    }
}
