using System;
using Unity.Entities;
using Unity.Mathematics.FixedPoint;

namespace Fixed.Transforms
{
    [Serializable]
    [WriteGroup(typeof(LocalToWorld))]
    [WriteGroup(typeof(LocalToParent))]
    public struct Translation : IComponentData
    {
        public fp3 Value;
    }
}
