directKinematics:
	- orocos component
	- it receives as input port the joint positions of the robot (from FRI)
	and calculate the forward kinematics of the robot, and send them as output ports.

Kinematic chain is defined for each segment by its position and its orientation with
respect to the frame just before. In Figure 2.13, all joints of Kuka LWR are represented in
A. Thus there are seven rotations according to Z-axis(0), Y-axis(1), Z-axis(2), Y-axis(3),
Z-axis(4), Y-axis(5),and Z-axis(6). For each segment of the robot, a frame is allocated.
This frame is composed by a position P (x, y, z) and an orientation R(R, P, Y ) (expressed
using Roll-Pitch-Yaw form

Those Frames are defined as (see B in Figure 2.13):
• F0 = (P (0, 0, 0), R(0, 0, 0)) : corresponds to the world frame.
• F1/0 = (P (0, 0, 0.31), R(0, 0, π)) : for the 1st segment, with Z-axis rotation.
• F2/1 = (P (0, 0, 0), R(0, 0, 0)) : for the 2nd segment, with Y-axis rotation.
• F3/2 = (P (0, 0, 0.4), R(0, 0, π)) : for the 3rd segment, with Z-axis rotation.
• F4/3 = (P (0, 0, 0), R(0, 0, 0)) : for the 4th segment, with Y-axis rotation.
• F5/4 = (P (0, 0, 0.39), R(0, 0, − π )) : for the 5th segment, with Z-axis rotation.
• F6/5 = (P (0, 0, 0), R(0, 0, 0)) : for the 6th segment, with X-axis rotation.
• F7/6 = (P (0, 0, 0), R(0, 0, 0)) : for the end effector, with Z-axis rotation.



