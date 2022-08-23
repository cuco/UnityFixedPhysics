using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Fixed.Mathematics;
using Fixed.Physics;
using UnityEngine;
using UnityEngine.Assertions;
using static BasicBodyInfo;
using static Fixed.Physics.Math;

public static class RagdollDemoUtilities
{
    public struct BodyInfo
    {
        public float3 Position;
        public quaternion Orientation;
        public float3 LinearVelocity;
        public float3 AngularVelocity;
        public sfloat Mass;
        public BlobAssetReference<Fixed.Physics.Collider> Collider;
        public bool IsDynamic;
    }

    public struct JointInfo
    {
        public int BodyIndexA;
        public int BodyIndexB;
        public PhysicsJoint JointData;
        public bool EnableCollision;
    }

    public static BodyInfo CreateBody(GameObject gameObject, List<BlobAssetReference<Fixed.Physics.Collider>> createdColliders)
    {
        var bounds = gameObject.GetComponent<MeshRenderer>().bounds;
        var basicBodyInfo = gameObject.GetComponent<BasicBodyInfo>();
        BlobAssetReference<Fixed.Physics.Collider> collider = default;

        switch (basicBodyInfo.Type)
        {
            case BodyType.Sphere:
                sfloat radius = math.cmax(bounds.extents);
                collider = Fixed.Physics.SphereCollider.Create(
                    new SphereGeometry
                    {
                        Center = float3.zero,
                        Radius = radius
                    });
                break;
            case BodyType.Box:
                collider = Fixed.Physics.BoxCollider.Create(
                    new BoxGeometry
                    {
                        Center = float3.zero,
                        Orientation = quaternion.identity,
                        Size = bounds.size,
                        BevelRadius = (sfloat)0.0f
                    });
                break;
            case BodyType.ConvexHull:
                var mesh = gameObject.GetComponent<MeshFilter>().mesh;
                var scale = gameObject.transform.lossyScale;
                NativeArray<float3> points = new NativeArray<float3>(mesh.vertexCount, Allocator.Temp);
                for (int i = 0; i < mesh.vertexCount; i++)
                {
                    points[i] = mesh.vertices[i];
                    points[i] *= scale;
                }
                ConvexHullGenerationParameters def = ConvexHullGenerationParameters.Default;
                def.BevelRadius = (sfloat)0.0f;
                collider = Fixed.Physics.ConvexCollider.Create(
                    points, def, CollisionFilter.Default);
                break;
            case BodyType.Capsule:
                var capsuleRadius = math.cmin(bounds.extents);
                var capsuleLength = math.cmax(bounds.extents);
                var capsuleGeometry = new CapsuleGeometry
                {
                    Radius = capsuleRadius,
                    Vertex0 = new float3(sfloat.Zero, capsuleLength - capsuleRadius, sfloat.Zero),
                    Vertex1 = new float3(sfloat.Zero, sfloat.MinusOne * (capsuleLength - capsuleRadius), sfloat.Zero)
                };
                collider = Fixed.Physics.CapsuleCollider.Create(capsuleGeometry);
                break;
            default:
                Assert.IsTrue(false, "Invalid body type");
                break;
        }


        createdColliders.Add(collider);
        bool isDynamic = !gameObject.GetComponent<BasicBodyInfo>().IsStatic;

        return new BodyInfo
        {
            Mass = isDynamic ? (sfloat)basicBodyInfo.Mass : (sfloat)0f,
            Collider = collider,
            AngularVelocity = float3.zero,
            LinearVelocity = float3.zero,
            Orientation = gameObject.transform.rotation,
            Position = gameObject.transform.position,
            IsDynamic = isDynamic
        };
    }

    public static PhysicsJoint CreateJoint(GameObject parentBody, GameObject childBody, BasicJointInfo.BasicJointType jointType)
    {
        var bodyPBounds = parentBody.GetComponent<MeshRenderer>().bounds;
        var bodyCBounds = childBody.GetComponent<MeshRenderer>().bounds;

        var pointConPWorld = bodyPBounds.ClosestPoint(bodyCBounds.center);
        var pointPonCWorld = bodyCBounds.ClosestPoint(bodyPBounds.center);

        var bodyPTransform = new RigidTransform(parentBody.transform.rotation, parentBody.transform.position);// was torso
        var bodyCTransform = new RigidTransform(childBody.transform.rotation, childBody.transform.position);// was head

        PhysicsJoint jointData = default;
        switch (jointType)
        {
            case BasicJointInfo.BasicJointType.BallAndSocket:
            {
                var pivotP = math.transform(math.inverse(bodyPTransform), pointConPWorld);
                var pivotC = math.transform(math.inverse(bodyCTransform), pointConPWorld);
                jointData = PhysicsJoint.CreateBallAndSocket(pivotP, pivotC);
            }
            break;
            case BasicJointInfo.BasicJointType.Distance:
            {
                var pivotP = math.transform(math.inverse(bodyPTransform), pointConPWorld);
                var pivotC = math.transform(math.inverse(bodyCTransform), pointPonCWorld);
                var range = new FloatRange(sfloat.Zero, math.distance(pointConPWorld, pointPonCWorld));
                jointData = PhysicsJoint.CreateLimitedDistance(pivotP, pivotC, range);
            }
            break;
            case BasicJointInfo.BasicJointType.Hinge:
            {
                var commonPivotPointWorld = math.lerp(pointConPWorld, pointPonCWorld, (sfloat)0.5f);

                // assume a vertical hinge joint
                var axisP = math.rotate(math.inverse(bodyPTransform.rot), math.up());
                var axisC = math.rotate(math.inverse(bodyCTransform.rot), math.up());

                float3 perpendicularAxisA, perpendicularAxisB;
                Math.CalculatePerpendicularNormalized(axisP, out perpendicularAxisA, out _);
                Math.CalculatePerpendicularNormalized(axisC, out perpendicularAxisB, out _);

                var pivotP = math.transform(math.inverse(bodyPTransform), commonPivotPointWorld);
                var pivotC = math.transform(math.inverse(bodyCTransform), commonPivotPointWorld);
                var jointFrameP = new BodyFrame { Axis = axisP, PerpendicularAxis = perpendicularAxisA, Position = pivotP };
                var jointFrameC = new BodyFrame { Axis = axisC, PerpendicularAxis = perpendicularAxisB, Position = pivotC };
                var range = new FloatRange(math.radians(-(sfloat)90), math.radians((sfloat)90.0f));
                jointData = PhysicsJoint.CreateLimitedHinge(jointFrameP, jointFrameC, range);
            }
            break;
            default:
                break;
        }
        return jointData;
    }
}
