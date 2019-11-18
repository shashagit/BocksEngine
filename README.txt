This engine was created in the process of learning to write a robust rigidbody simulation while taking the CS 550 course at DigiPen. It was later heavily improved upon and extended to include Joint constraints as well.

INSTRUCTIONS:
Move Camera using WASD
Right Click and move Mouse to Rotate Camera
Press P to Pause/Play the Simulation
Press F to go to Next Frame when Paused
Press N to spawn objects at a fixed position

NOTES:
-Components/Body.h has the Rigidbody interface in it
-Components/Collider.h has the Half-edge data structure implementation
-Managers/PhysicsSystem.cpp has the Physics Step
-Broadphase/ has the DynamicAABB and NSquared implementation in it
-Narrowphase/ has SAT.cpp which has the entire SAT code with clipping
-Articulation/ has the Joint constraint classes
