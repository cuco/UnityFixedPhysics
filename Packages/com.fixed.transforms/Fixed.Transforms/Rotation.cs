using System;
using Unity.Entities;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Transforms
{
    [Serializable]
    [WriteGroup(typeof(LocalToWorld))]
    [WriteGroup(typeof(LocalToParent))]
    [WriteGroup(typeof(CompositeRotation))]
    public struct Rotation : IComponentData
    {
        public fpquaternion Value;
    }
}
