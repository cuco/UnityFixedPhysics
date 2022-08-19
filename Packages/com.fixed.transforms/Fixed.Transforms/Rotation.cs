using System;
using Unity.Entities;
using Fixed.Mathematics;

namespace Fixed.Transforms
{
    [Serializable]
    [WriteGroup(typeof(LocalToWorld))]
    [WriteGroup(typeof(LocalToParent))]
    [WriteGroup(typeof(CompositeRotation))]
    public struct Rotation : IComponentData
    {
        public quaternion Value;
    }
}
