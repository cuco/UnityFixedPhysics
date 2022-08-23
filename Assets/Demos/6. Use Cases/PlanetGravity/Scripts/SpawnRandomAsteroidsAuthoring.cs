using System;
using Unity.Entities;
using Fixed.Physics;
using Fixed.Mathematics;
using UnityEngine;
using Random = Fixed.Mathematics.Random;

class SpawnRandomAsteroidsAuthoring : SpawnRandomObjectsAuthoringBase<AsteroidSpawnSettings>
{
    public sfloat massFactor = sfloat.One;

    internal override void Configure(ref AsteroidSpawnSettings spawnSettings) => spawnSettings.MassFactor = massFactor;
}

struct AsteroidSpawnSettings : IComponentData, ISpawnSettings
{
    public Entity Prefab { get; set; }
    public float3 Position { get; set; }
    public quaternion Rotation { get; set; }
    public float3 Range { get; set; }
    public int Count { get; set; }
    public sfloat MassFactor;
}

class SpawnRandomAsteroidsSystem : SpawnRandomObjectsSystemBase<AsteroidSpawnSettings>
{
    Random m_RandomMass;

    internal override int GetRandomSeed(AsteroidSpawnSettings spawnSettings)
    {
        var seed = base.GetRandomSeed(spawnSettings);
        seed = (seed * 397) ^ spawnSettings.Prefab.GetHashCode();
        seed = (seed * 397) ^ (int)(spawnSettings.MassFactor * (sfloat)1000);
        return seed;
    }

    internal override void OnBeforeInstantiatePrefab(ref AsteroidSpawnSettings spawnSettings)
    {
        m_RandomMass = new Random();
        m_RandomMass.InitState(10);
    }

    internal override void ConfigureInstance(Entity instance, ref AsteroidSpawnSettings spawnSettings)
    {
        var mass = EntityManager.GetComponentData<PhysicsMass>(instance);
        var halfMassFactor = spawnSettings.MassFactor * (sfloat)0.5f;
        mass.InverseMass = m_RandomMass.NextFloat(mass.InverseMass * math.rcp(halfMassFactor), mass.InverseMass * halfMassFactor);
        EntityManager.SetComponentData(instance, mass);
    }
}
