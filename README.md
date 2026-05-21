# torsobot
github repository for torsobot

The design folder contains CAD and ECAD files. The BOM is embedded inside the main SolidWorks assembly file (Torsobot_asm.sldasm). 
There are two main MATLAB folders named “2DRimlessWheel” and “torsobot_dynamics” respectively. The first one has files for simulating the 2D rimless wheel model; the second one has the simulation model and scripts for modeling the Torsobot. For the Torsobot, the parameters.m file contains the parameters for the robot and controller and the main simulation is run from main.m. The speed controller is run from speedControlSimulation.m.
The microcontroller code is under the folder Arduino and contains the .ino (motor_torque_control_dualcore.ino) and .h files. It is important to have the following version of libraries especially for Moteus.
1.	PicoEncoder byPaulo Marques (v1.1.1)
2.	Moteus by info@mjbots.com (v1.0.2)
3.	ACAN2517FD by Pierre Molinaro (v2.1.16)
4.	Adafruit BNO08x by Adafruit (v1.2.5)
