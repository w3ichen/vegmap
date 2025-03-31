odel Elements

Model: The top-level container for a physical entity
Link: Rigid bodies that make up a model
Joint: Connections between links that constrain their relative movement
Collision: Geometry used for collision detection
Visual: Geometry used for visualization
Sensor: Elements that can generate data from the simulation
Plugin: Code modules that extend functionality

Joint Types
- Fixed: Rigidly connects two links
- Revolute: Rotation around a single axis (hinge)
- Prismatic: Linear movement along a single axis
- Screw: Combined rotation and translation along the same axis
- Universal: Rotation around two perpendicular axes
- Ball: Rotation around all three axes (spherical joint)
- Gearbox: Connects two revolute joints with a gearing ratio

Physical Properties
For Links:

Mass: Weight of the link
Inertia: Resistance to changes in rotational motion
Self-collision: Whether collision detection is enabled between links of the same model

For Collisions:

Friction: Both static and dynamic friction coefficients
Restitution: Bounciness of collisions
Contact parameters: Surface softness, penetration depth, etc.
Surface: Properties like bounce, friction, and contact parameters

For Joints:

Axis: Direction of movement/rotation
Limits: Range of motion constraints
Dynamics: Damping, friction, and spring stiffness
Effort: Maximum force/torque that can be applied

Sensor Types

Camera: Generates images
Ray/Lidar: Simulates laser scanners
IMU: Measures acceleration and orientation
Contact: Detects collisions
Force/Torque: Measures forces and torques at joints
GPS: Provides global positioning
Logical Camera: Provides object identification and pose

Other Notable Elements

Material: Visual properties like texture and color
Light: Light sources in the environment
Actor: Special entity for scripted motion
State: Storing and restoring simulation states
World: Container for models, physics properties, and environment settings