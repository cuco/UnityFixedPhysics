using Unity.Entities;
using Fixed.Mathematics;
using UnityEngine;

namespace Fixed.Physics.Authoring
{
    [RequireComponent(typeof(PhysicsBodyAuthoring))]
    public abstract class BaseBodyPairConnector : MonoBehaviour
    {
        public PhysicsBodyAuthoring LocalBody => GetComponent<PhysicsBodyAuthoring>();
        public PhysicsBodyAuthoring ConnectedBody;

        public RigidTransform worldFromA => LocalBody == null
        ? RigidTransform.identity
        : Math.DecomposeRigidBodyTransform(LocalBody.transform.localToWorldMatrix);

        public RigidTransform worldFromB => ConnectedBody == null
        ? RigidTransform.identity
        : Math.DecomposeRigidBodyTransform(ConnectedBody.transform.localToWorldMatrix);


        public Entity EntityA { get; set; }

        public Entity EntityB { get; set; }


        void OnEnable()
        {
            // included so tick box appears in Editor
        }

        public abstract void Create(EntityManager entityManager, GameObjectConversionSystem conversionSystem);
    }
}
