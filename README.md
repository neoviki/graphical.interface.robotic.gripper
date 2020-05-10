#### Teensy Robotic Gripper

This code can be used to control a custom robottic gripper I am working on. The gripper can be controlled using serial communication interface. 

Platform Supported:

	- Teensy 3.2
	- Arduino Uno, Mega

Hardware Used:

	- 3D printed robotic gripper 
	- Motor	Type	: DC Motor [ 12V - 8.6 RPM ]
	- Encoder Type	: Quadrature Encoder

During startup the code does the following, 

1. Opens the gripper till the gripper opening limit
2. Close the gripper till the gripper closing limit
3. Open and Close limit of the gripper is computed using quadrature encoder readings.

Serial Command Reference:

1. "o"        : fully open gripper
2. "c"        : fully close gripper
3. "ap<xxx>"  : actuate gripper by a percentage(xxx)
4. "OL<xxx>"  : Set open Limit for gripper by a value (xxx)
5. "CL<xxx>"  : Set close limit for gripper by a value (xxx)
6. "GO<xxx>"  : Set Gripper Open Factor by a value (xxx)
7. "GC<xxx>"  : Set Gripper Close Factor by a value (xxx)

The gripper is fitted with a brushless dc motor with quadrature encoder.
