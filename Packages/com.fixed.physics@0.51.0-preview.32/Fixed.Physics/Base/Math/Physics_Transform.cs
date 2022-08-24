using System;
using System.Diagnostics;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Physics
{
    public static partial class Math
    {
        [DebuggerStepThrough]
        public struct MTransform : IEquatable<MTransform> // TODO: Replace with Unity fp4x4 ?
        {
            public fp3x3 Rotation;
            public fp3 Translation;

            public static MTransform Identity => new MTransform { Rotation = fp3x3.identity, Translation = fp3.zero };

            public MTransform(FpRigidTransform transform)
            {
                Rotation = new fp3x3(transform.rot);
                Translation = transform.pos;
            }

            public MTransform(fpquaternion rotation, fp3 translation)
            {
                Rotation = new fp3x3(rotation);
                Translation = translation;
            }

            public MTransform(fp3x3 rotation, fp3 translation)
            {
                Rotation = rotation;
                Translation = translation;
            }

            public fp3x3 InverseRotation => fpmath.transpose(Rotation);

            public bool Equals(MTransform other)
            {
                return this.Rotation.Equals(other.Rotation) && this.Translation.Equals(other.Translation);
            }
        }

        public static fp3 Mul(MTransform a, fp3 x)
        {
            return fpmath.mul(a.Rotation, x) + a.Translation;
        }

        // Returns cFromA = cFromB * bFromA
        public static MTransform Mul(MTransform cFromB, MTransform bFromA)
        {
            return new MTransform
            {
                Rotation = fpmath.mul(cFromB.Rotation, bFromA.Rotation),
                Translation = fpmath.mul(cFromB.Rotation, bFromA.Translation) + cFromB.Translation
            };
        }

        public static MTransform Inverse(MTransform a)
        {
            fp3x3 inverseRotation = fpmath.transpose(a.Rotation);
            return new MTransform
            {
                Rotation = inverseRotation,
                Translation = fpmath.mul(inverseRotation, -a.Translation)
            };
        }
    }
}
