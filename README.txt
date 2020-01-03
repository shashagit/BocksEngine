README:

INSTRUCTIONS:
Move Camera using WASD
Right Click and move Mouse to Rotate Camera
Press P to Pause/Play the Simulation
Press F to go to Next Frame when Paused
Press N to spawn objects at a fixed position

NOTES:
-Please run the project on Release to get optimum performance
-For all levels with 2 or less boxes, each box can be moved by using
 the Velocity and Angular Velocity slider in the ImGui window


-Components/Body.h has the Rigidbody interface in it
-Components/Collider.h has the Half-edge data structure implementation
-Managers/PhysicsSystem.cpp has the Physics Step
-Broadphase/ has the DynamicAABB and NSquared implementation in it
-Narrowphase/ has SAT.cpp which has the entire SAT code with clipping
-Articulation/Joint.h has the Joints implementation in it

