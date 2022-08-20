# Contact jacobian memory layout

| **Start byte**                  | **End byte** | **Member name**                             |
----------------------------------|--------------|---------------------------------------------|
 **JacobianHeader**               |              |                                             |
 0                                | 7            | BodyPair (2 ints)                           |
 8                                | 8            | Type (byte)                                 |
 9                                | 9            | JacModFlags (byte)                          |
 **BaseContactJacobian**          |              |                                             |
 0                                | 3            | NumContacts (int)                           |
 4                                | 15           | Normal (float3)                             |
 **ContactJacobianAngular**       |              |                                             |
 0                                | 11           | AngularA (float3)                           |
 12                               | 23           | AngularB (float3)                           |
 24                               | 27           | EffectiveMass (sfloat)                       |
 28                               | 31           | Impulse (sfloat)                             |
 **ContactJacAngAndVelToReachCP** |              |                                             |
 0                                | 31           | Jac (ContactJacobianAngular)                |
 32                               | 35           | VelToReachCP (sfloat)                        |
 **ContactJacobian**              |              |                                             |
 0                                | 15           | BaseJac (BaseContactJacobian)               |
 16                               | 19           | CoefficientOfFriction (sfloat)               |
 20                               | 23           | CoefficientOfRestitution (sfloat)            |
 24                               | 55           | Friction0 (ContactJacobianAngular)          |
 56                               | 87           | Friction1 (ContactJacobianAngular)          |
 88                               | 91           | FrictionEffectiveMassNonDiag (sfloat)        |
 92                               | 123          | AngularFriction (ContactJacobianAngular)    |
 **TriggerJacobian**              |              |                                             |
 0                                | 15           | BaseJac (BaseContactJacobian)               |
 16                               | 23           | ColliderKeys (2 ints)                       |
 **ContactPoint**
 0                                | 11           | Position (float3)                           |
 12                               | 15           | Distance (sfloat)                            |

  **Contact jacobian types**                      |
--------------------------------------------------|
 **JacobianType.Contact**                         |
 JacobianHeader                                   |
 ContactJacobian                                  |
 Modifier data (based on JacModFlags)             |
 NumContacts * ContactJacAngAndVelToReachCP       |
 NumContacts * ContactPoint (based on JacModFlags)|
 **JacobianType.Trigger**                         |
 JacobianHeader                                   |
 TriggerJacobian                                  |
 NumContacts * ContactJacAngAndVelToReachCP       |