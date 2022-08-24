using System;
using Unity.Entities;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Transforms
{
    [Serializable]
    [WriteGroup(typeof(WorldToLocal))]
    public struct LocalToWorld : IComponentData
    {
        public fp4x4 Value;

        public fp3 Right => new fp3(Value.c0.x, Value.c0.y, Value.c0.z);
        public fp3 Up => new fp3(Value.c1.x, Value.c1.y, Value.c1.z);
        public fp3 Forward => new fp3(Value.c2.x, Value.c2.y, Value.c2.z);
        public fp3 Position => new fp3(Value.c3.x, Value.c3.y, Value.c3.z);

        public fpquaternion Rotation => new fpquaternion(Value);
    }
}
