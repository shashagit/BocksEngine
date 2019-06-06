README:

INSTRUCTIONS:
Move Camera using WASD
Right Click and move Mouse to Rotate Camera
Press P to Pause/Play the Simulation
Press F to go to Next Frame when Paused
Press N to spawn objects at a fixed position

NOTES:
-Please run the project on Release 
-Checkerboard boxes have fixed position and do not collide among themselves through a hack
-n-squared broadphase is also implemented 
	(comment lines 33, 59, 62, 63 and uncomment lines 35, 55, 56 in Managers/PhysicsSystem.cpp to run)

-Components/Body.h has the Rigidbody interface in it
-Components/Collider.h has the Half-edge data structure implementation
-Managers/PhysicsSystem.cpp has the Physics Step
-Broadphase/ has the DynamicAABB and NSquared implementation in it
-Narrowphase/ has SAT.cpp which has the entire SAT code with clipping

