using System;
using Unity.Mathematics;
using Unity.Mathematics.FixedPoint;
using UnityEngine;

namespace Fixed.Physics.Authoring
{
    [Serializable]
    struct EulerAngles : IEquatable<EulerAngles>
    {
        public static EulerAngles Default => new EulerAngles { RotationOrder = math.RotationOrder.ZXY };

        public fp3 Value;
        [HideInInspector]
        public math.RotationOrder RotationOrder;

        internal void SetValue(fpquaternion value) => Value = fpmath.degrees(Math.ToEulerAngles(value, RotationOrder));

        public static implicit operator fpquaternion(EulerAngles euler) =>
            fpmath.normalize(fpquaternion.Euler(fpmath.radians(euler.Value), euler.RotationOrder));

        public bool Equals(EulerAngles other) => Value.Equals(other.Value) && RotationOrder == other.RotationOrder;

        public override bool Equals(object obj) => obj is EulerAngles other && Equals(other);

        public override int GetHashCode() => unchecked((int)fpmath.hash(new fp4(Value, (fp)(int)RotationOrder)));
    }
}
