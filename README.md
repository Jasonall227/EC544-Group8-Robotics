# EC544-Group8-Robotics

## The Robotics Part of EC544 Group Project

The robot consists of a FRDM-K64F board, a robotic arm kit purchased on Amazon, a PCA9685 actuation module, and the wiring system. It can interact with other sections of the project, i.e. the Raspberry pi.

The FRDM-K64F board generates signals to control the six MG996 rudder motors of the robotic arm. It also receives input signals from a Raspberry pi microcomputer. The board provides multiple GPIO ports, and has a real-time operating system capable of generating high-precision software PWM signal.  

The motors have three inputs: Power source GND, Power source VCC, and the control signal. The control signal is a 0.5ms to 2.5ms high level pulse in 20ms periods. The motor will rotate to and remain at the corresponding position(0 degree with 0.5ms, 200 degrees with 2.5ms) if the given control signal is valid, and will not move or hold its position if the high level pulse exceeds the given period. The precision requirement of the PWM control signal requires a real-time operating system, which means using a Raspberry pi to control the motors is impossible(it has a non-real-time operating system).

The actuation module serves as separation and integration of the wiring system. The power input pins of the motors are all connected to the module, which provides convenient pin ports. However the PWM signal ports of the module is joint, which means one PWM signal controls all motors on the actuation module, if the PWM ports of the motors are connected to it. To ensure different PWM signals available for all six motors, the control signal is given directly from the FRDM board to the motors.

## Software Coding

The NXP official SDK is used with the assistance of example code. 

To generate one single high level pulse in every 20ms, the Periodic Interrupt Timer module of the FRDM board is used. It generates a custom periodic interrupt. This serves as the major part of the main loop in the code. 

In every 20ms period, the board generates the corresponding high level pulses for all six motors sequentially. Since each requires less than 2.5ms compared to 20ms, there is enough time and margin to do them sequentially with delay functions.

As for the external control signal (from the Raspberry pi, for example), we provide two ways of receiving them. One is to check for an ENABLE signal input from a GPIO port at every start of the 20ms period, the other is to use external GPIO interrupt. 5 pins are connected to the FRDM board as external control signal. There are three ways of control:

1. The step control. Each time an instruction is given at the Raspberry control center, the robotic arm will receive it and rotate a corresponding motor by a certain degree.
2. The consistant control. When an instruction is given, the corresponding motor will begin to rotate continuously in the designated direction, and will not stop until a stop signal is received.
3. The position control. Each time an instruction is given, the FRDM board will read a motor selection signal and a position signal, and rotate the corresponding motor to the designated position. There does not need to be a stop signal; one single instruction from the Raspberry pi will be enough.

Each of the above could be with or without the external interrupt function. However, because of bad quality wires and concerns about the interferance between two interrupt sources(external GPIO vs periodic timer), we mainly focused on the no-external-interrupt way.



## Weekly Achievements

Week 1.  Purchased the components, set up the development environment, downloaded needed resourses, assembled the robotic arm, and started working on the project.

Week 2.  Successfully carried out a test on one motor. Achieved basic PWM control on motors. Completed the Periodic Interrupt Timer section.

Week 3.  Expanded and modified the drive code for motors. Added a gentle rotation function. 
Since there is no feedback from the motor, there is no knowing which position the motor is at, except when an instruction has been given and the motors have rotated to the designated position. The gentle function makes the rotation from one known position to another more gentle, instead of simply sending the new destination and letting the motors rotate too acutely.

Week 4.  Completed all drive for motors. Connected all the wires. Looked deeper into GPIO functions on the FRDM board.

Week 5.  Completed control for motors. Recorded a demo. Optimization of the code. Completed all GPIO configurations and initializations.

Week 6.  Calibration of the parameters. Optimization of code. 

Week 7.  Building interactions with other sections of the project. Interaction with the Raspberry pi. Final calibration of the interaction and parameters.

## Conclusions

Not all goals in the proposal are met. If there had been more time, that should be fine.

Not many problems expected in the proposal are encountered. There are a few problems that we did not expect in the proposal, including the vagueness of the official SDK, the time it takes to solve connection problems, the unstable signal from devices, etc. 

Apart from the specific knowledge acquired during the project, another important thing is the ability to look into instructions in-depth in official head files. The official SDK does not provide clear information, the user manual provides little insight for programming, and the online resources are relatively sparse. When working through the official example SDK code, it is important but somewhat challenging to make sense of what one parameter means, what type of parameter should be given to a function, and where to find the useful functions. The project has strengthened the ability of fast development, and the ability of finding the required function quickly and accurately within ill-noted example programs.

The knowledge from the first section of EC544 and the experience gained from the project are well-combined. We had a chance to look into re-entrancy, real-time and non-real-time operation systems, and interrupt privileges. It is the methodology I acquired or strengthened in this project that interests me most.
