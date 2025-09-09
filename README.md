# Trajectory-Tracking-in-Unity-Engine
A project connecting Unity and Python for autonomous system simulations. Unity working as the simulator tool while Python acts as the data processor. **This Github page is a work in progress!!!**

## How this project works: The Unity to Python to Unity Circuit

### Unity Engine
- Sphere acts as the autonomous system being controlled and uses Unity's rigid body physics.
- Unity will provide ground truth data on the sphere's position (x y z) and velocity (vx vy vz) to Python.

### Python
- Measurement Function: Takes in ground truth data from Unity, adds Gaussian noise.
- State Estimator: Takes in Measuremment Function data, produces optimal state estimation.
- Reference Function: Provides reference coordinates as targets for the autonomous system to reach.
- Controller: Takes in optimal state estimate, outputs control command that is sent to Unity.

### Unity Engine
- Receives velocity control command from Python (ux uy uz) that Unity then executes on the sphere.
- Unity will then provide ground truth data, beginning the circuit again. 
