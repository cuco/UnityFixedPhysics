using Unity.Entities;
using UnityEngine;

public class GravityWellComponentAuthoring_DOTS : MonoBehaviour, IConvertGameObjectToEntity
{
    public sfloat Strength = (sfloat)100.0f;
    public sfloat Radius = (sfloat)10.0f;

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        dstManager.AddComponentData(entity, new GravityWellComponent_DOTS
        {
            Strength = Strength,
            Radius = Radius,
            Position = gameObject.transform.position
        });
    }
}
