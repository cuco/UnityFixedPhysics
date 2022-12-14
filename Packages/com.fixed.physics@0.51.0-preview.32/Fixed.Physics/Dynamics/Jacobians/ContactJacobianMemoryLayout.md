# Contact jacobian memory layout

| **Start byte**                  | **End byte** | **Member name**                             |
----------------------------------|--------------|---------------------------------------------|
 **JacobianHeader**               |              |                                             |
 0                                | 7            | BodyPair (2 ints)                           |
 8                                | 8            | Type (byte)                                 |
 9                                | 9            | JacModFlags (byte)                          |
 **BaseContactJacobian**          |              |                                             |
 0                                | 3            | NumContacts (int)                           |
 4                                | 15           | Normal (fp3)                             |
 **ContactJacobianAngular**       |              |                                             |
 0                                | 11           | AngularA (fp3)                           |
 12                               | 23           | AngularB (fp3)                           |
 24                               | 27           | EffectiveMass (fp)                       |
 28                               | 31           | Impulse (fp)                             |
 **ContactJacAngAndVelToReachCP** |              |                                             |
 0                                | 31           | Jac (ContactJacobianAngular)                |
 32                               | 35           | VelToReachCP (fp)                        |
 **ContactJacobian**              |              |                                             |
 0                                | 15           | BaseJac (BaseContactJacobian)               |
 16                               | 19           | CoefficientOfFriction (fp)               |
 20                               | 23           | CoefficientOfRestitution (fp)            |
 24                               | 55           | Friction0 (ContactJacobianAngular)          |
 56                               | 87           | Friction1 (ContactJacobianAngular)          |
 88                               | 91           | FrictionEffectiveMassNonDiag (fp)        |
 92                               | 123          | AngularFriction (ContactJacobianAngular)    |
 **TriggerJacobian**              |              |                                             |
 0                                | 15           | BaseJac (BaseContactJacobian)               |
 16                               | 23           | ColliderKeys (2 ints)                       |
 **ContactPoint**
 0                                | 11           | Position (fp3)                           |
 12                               | 15           | Distance (fp)                            |

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