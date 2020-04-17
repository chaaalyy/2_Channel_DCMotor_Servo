# 2_Channel_DCMotor_Servo
Two channel brushed DC-Motor Servo control for step-dir signals with Nucleo-F401RE and external PWM motor driver

If you need to control brushed DC Motors with step/dir signals, commonly used for steppers / servos in cnc machines or industrial environments, maybe this code can help you.

My own parts of the code are licensed under GNU GENERAL PUBLIC LICENSE V3.0

I added two config jumpers to give you more flexibility:
J1 decides, if two or one input channels should be used (one: two motors synchronized, two: each motor controlled separately).
J2 in single channel mode: which channel to use, in two channel mode: you can swap the channels.
Of course, it´s also possible to control just one motor: config dual channel and just don´t use the 2nd one ;)

Created with ST´s cubeIDE 1.3 , hardware config via integrated cubeMX. Unused GPIOs are all set to GPIO_output without further label to get better emf stability. Used pins are marked and should be clearly visible via the cubeMX configuration.

Used hardware: Board = Nucleo F401RE, Motor Driver = Cytron MDD10A, 2x geared (70:1) Motor Pololu #4694 w. integrated incremental Encoder (4480 ppRev)

This is my 1st public repository and i´m not a full time software engineer ;) So be gracious, when it comes to coding style or any mistakes :)

used external Libs (slightly modified to work with f401 mcu ;) ):
/**********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/
