using Unity.Mathematics.FixedPoint;
using UnityEngine;

namespace Fixed.Physics.Authoring
{
    /// <summary>
    /// Describes how a rigid body will be simulated in the run-time.
    /// </summary>
    public enum BodyMotionType
    {
        /// <summary>
        /// The physics solver will move the rigid body and handle its collision response with other bodies, based on its physical properties.
        /// </summary>
        Dynamic,
        /// <summary>
        /// The physics solver will move the rigid body according to its velocity, but it will be treated as though it has infinite mass.
        /// It will generate a collision response with any rigid bodies that lie in its path of motion, but will not be affected by them.
        /// </summary>
        Kinematic,
        /// <summary>
        /// The physics solver will not move the rigid body.
        /// Any transformations applied to it will be treated as though it is teleporting.
        /// </summary>
        Static
    }

    /// <summary>
    /// Describes how a rigid body's motion in its graphics representation should be smoothed when the rendering framerate is greater than the fixed step rate used by physics.
    /// </summary>
    public enum BodySmoothing
    {
        /// <summary>
        /// The body's graphics representation will display its current position and orientation from the perspective of the physics solver.
        /// </summary>
        None,
        /// <summary>
        /// The body's graphics representation will display a smooth result between the two most recent physics simulation ticks.
        /// The result is one tick behind, but will not mis-predict the body's position and orientation.
        /// However, it can make the body appear as if it changes direction before making contact with other bodies, particularly when the physics tick rate is low.
        /// See <seealso cref="GraphicsIntegration.GraphicalSmoothingUtility.Interpolate"/> for details.
        /// </summary>
        Interpolation,
        /// <summary>
        /// The body's graphics representation will display a smooth result by projecting into the future based on its current velocity.
        /// The result is thus up-to-date, but can mis-predict the body's position and orientation since any future collision response has not yet been resolved.
        /// See <seealso cref="GraphicsIntegration.GraphicalSmoothingUtility.Extrapolate"/> for details.
        /// </summary>
        Extrapolation
    }

    [AddComponentMenu("DOTS/Physics/Physics Body")]
    [DisallowMultipleComponent]
    [HelpURL(HelpURLs.PhysicsBodyAuthoring)]
    public sealed class PhysicsBodyAuthoring : MonoBehaviour
    {
        PhysicsBodyAuthoring() {}

        public BodyMotionType MotionType { get => m_MotionType; set => m_MotionType = value; }
        [SerializeField]
        [Tooltip("Specifies whether the body should be fully physically simulated, moved directly, or fixed in place.")]
        BodyMotionType m_MotionType;

        public BodySmoothing Smoothing { get => m_Smoothing; set => m_Smoothing = value; }
        [SerializeField]
        [Tooltip("Specifies how this body's motion in its graphics representation should be smoothed when the rendering framerate is greater than the fixed step rate used by physics.")]
        BodySmoothing m_Smoothing = BodySmoothing.None;

        //TODO
        static readonly fp k_MinimumMass = (fp)0.001f;

        public fp Mass
        {
            get => m_MotionType == BodyMotionType.Dynamic ? m_Mass : fp.PositiveInfinity;
            set => m_Mass = fpmath.max(k_MinimumMass, value);
        }
        [SerializeField]
        fp m_Mass = fp.one;

        public fp LinearDamping { get => m_LinearDamping; set => m_LinearDamping = fpmath.max(fp.zero, value); }
        [SerializeField]
        [Tooltip("This is applied to a body's linear velocity reducing it over time.")]
        fp m_LinearDamping = (fp)0.01f;

        public fp AngularDamping { get => m_AngularDamping; set => m_AngularDamping = fpmath.max(fp.zero, value); }
        [SerializeField]
        [Tooltip("This is applied to a body's angular velocity reducing it over time.")]
        fp m_AngularDamping = (fp)0.05f;

        public fp3 InitialLinearVelocity { get => m_InitialLinearVelocity; set => m_InitialLinearVelocity = value; }
        [SerializeField]
        [Tooltip("The initial linear velocity of the body in world space")]
        fp3 m_InitialLinearVelocity = fp3.zero;

        public fp3 InitialAngularVelocity { get => m_InitialAngularVelocity; set => m_InitialAngularVelocity = value; }
        [SerializeField]
        [Tooltip("This represents the initial rotation speed around each axis in the local motion space of the body i.e. around the center of mass")]
        fp3 m_InitialAngularVelocity = fp3.zero;

        public fp GravityFactor
        {
            get => m_MotionType == BodyMotionType.Dynamic ? m_GravityFactor : fp.zero;
            set => m_GravityFactor = value;
        }
        [SerializeField]
        [Tooltip("Scales the amount of gravity to apply to this body.")]
        fp m_GravityFactor = fp.one;

        public bool OverrideDefaultMassDistribution
        {
#pragma warning disable 618
            get => m_OverrideDefaultMassDistribution;
            set => m_OverrideDefaultMassDistribution = value;
#pragma warning restore 618
        }
        [SerializeField]
        [Tooltip("Default mass distribution is based on the shapes associated with this body.")]
        bool m_OverrideDefaultMassDistribution;

        public MassDistribution CustomMassDistribution
        {
            get => new MassDistribution
            {
                Transform = new FpRigidTransform(m_Orientation, m_CenterOfMass),
                InertiaTensor =
                    m_MotionType == BodyMotionType.Dynamic ? m_InertiaTensor : new fp3(fp.PositiveInfinity)
            };
            set
            {
                m_CenterOfMass = value.Transform.pos;
                m_Orientation.SetValue(value.Transform.rot);
                m_InertiaTensor = value.InertiaTensor;
#pragma warning disable 618
                m_OverrideDefaultMassDistribution = true;
#pragma warning restore 618
            }
        }

        [SerializeField]
        fp3 m_CenterOfMass;

        [SerializeField]
        EulerAngles m_Orientation = EulerAngles.Default;

        [SerializeField]
        // Default value to solid unit sphere : https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        fp3 m_InertiaTensor = new fp3((fp)2f / (fp)5f);

        public uint WorldIndex { get => m_WorldIndex; set => m_WorldIndex = value; }
        [SerializeField]
        [Tooltip("The index of the physics world this body belongs to. Default physics world has index 0.")]
        uint m_WorldIndex = 0;

        public CustomPhysicsBodyTags CustomTags { get => m_CustomTags; set => m_CustomTags = value; }
        [SerializeField]
        CustomPhysicsBodyTags m_CustomTags = CustomPhysicsBodyTags.Nothing;

        void OnEnable()
        {
            // included so tick box appears in Editor
        }

        void OnValidate()
        {
            m_Mass = fpmath.max(k_MinimumMass, m_Mass);
            m_LinearDamping = fpmath.max(m_LinearDamping, fp.zero);
            m_AngularDamping = fpmath.max(m_AngularDamping, fp.zero);
        }
    }
}
