using Unity.Entities;
using Fixed.Mathematics;
using UnityEngine;

public class GravityWellComponent_GO : MonoBehaviour, IConvertGameObjectToEntity
{
    public sfloat Strength = (sfloat)100.0f;
    public sfloat Radius = (sfloat)10.0f;

    #region ECS
    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        dstManager.AddComponentData(entity, new GravityWellComponent_GO_ECS
        {
            Strength = Strength,
            Radius = Radius,
            Position = gameObject.transform.position
        });
    }

    #endregion
}

#region ECS
public struct GravityWellComponent_GO_ECS : IComponentData
{
    public sfloat Strength;
    public sfloat Radius;
    // Include position of gravity well so all data accessible in one location
    public float3 Position;
}
#endregion
