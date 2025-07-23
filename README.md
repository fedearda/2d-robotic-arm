# 2d-robotic-arm
Simulation of a robotic arm with n-joints. Objective: reach target in max MAX_ITERATIONS

# Parameters
- THRESH: is the threshold under the which the algorithm stops and considers the target reached - it is compared at each iteration with the distance between end-effector and target
- MAX_ANGLE_PER_STEP: this constraint is due to the fact that real implementations would require a limitation on the angle to be actuated for each joint at each iteration - angle is clamped to this value
- MAX_ITERATIONS: is the maximum values of iteration - if the target is not reached in MAX_ITERATIONS, then the target cannot be reached
- DAMP_THRESHOLD: is the threshold of distance at which the damping is applied - simulation of a real application
- DAMP_FACTOR: is a factor <1 that is applied to the actuated angle when the target is approached
- VERBOSITY_LEVEL: find info at the header of classes.hpp

# Screenshots
Example with target: {1.5,0.1} and VERBOSITY_LEVEL = INFO

<img width="634" height="619" alt="image" src="https://github.com/user-attachments/assets/2dbf3c8b-9dd4-46ff-aab3-91d198b99cdb" />

...


<img width="634" height="602" alt="image" src="https://github.com/user-attachments/assets/aabf016b-1be2-4946-8d9f-3badaad01083" />

