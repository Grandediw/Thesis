# Thesis: Torque Control of a DC Motor Using PID Regulator

## Overview

This repository contains the implementation and documentation for my Bachelor thesis titled **"Torque Control of a DC Motor Using PID Regulator"**, completed at the University of Trento in the academic year 2022-2023. The main objective of this thesis is to design a PID controller to regulate the torque of a DC motor, controlled via a microcontroller.

## Abstract

The purpose of this project is to develop a torque control system for a DC motor using a PID controller, with the ultimate goal of finding optimal parameters for the proportional, integral, and derivative components of the controller. The project involves:

1. Designing the PID regulator.
2. Implementing the controller on a microcontroller (Arduino).
3. Experimenting with different tuning techniques to optimize the controller's performance.
4. Validating the results through hardware implementation.

This thesis is divided into several chapters covering the theoretical aspects, software implementation, and hardware setup used to achieve these goals.

## Structure

The project is structured as follows:

- **Chapter 1: Introduction**  
  Overview of electrical drives and their applications, focusing on the closed-loop control of a DC motor.

- **Chapter 2: DC Motors and Drives**  
  Detailed analysis of DC motor operation, dynamic models, and driving techniques (ON-OFF, PWM control).

- **Chapter 3: System Identification and Control**  
  Explanation of the system identification process and the design of the PID controller. A software-based identification using MATLAB's System Identification Toolbox is carried out to obtain the motor's transfer function.

- **Chapter 4: Hardware Implementation and Results**  
  Description of the hardware components used, including Arduino UNO, Pololu Dual VNH5019 Motor Driver Shield, INA219 current sensor, and the DC motor with encoder. Experimental results are presented for system identification and controller performance evaluation.

- **Chapter 5: Conclusions and Future Work**  
  Reflection on the outcomes of the project and potential directions for future research.

## Hardware and Software

### Hardware Components:
- Arduino UNO
- Pololu Dual VNH5019 Motor Driver Shield
- INA219 Current Sensor
- DC Motor with Encoder

### Software:
- Arduino IDE
- MATLAB (System Identification Toolbox, PID Tuner)
- Simulink (for PID tuning)

## Results

The key findings of this thesis include the successful identification of the motor’s transfer function and the development of a robust PID controller for both torque and speed regulation. The controller parameters were tuned using various methods (manual tuning, Ziegler-Nichols, and MATLAB’s PID Tuner) and tested on real hardware. The performance of the system was evaluated through step response analysis and experimental testing.

## Code and Implementation

The repository includes:
- Arduino code for torque and speed control.
- MATLAB scripts for system identification and communication with the Arduino board.
- Experimental results and graphs generated during testing.

## Future Development

Future improvements could involve applying this control method to motors with different characteristics or exploring advanced control techniques beyond PID. Additionally, the setup could be extended to more complex systems requiring multi-motor coordination or integration with higher-level control systems.

## Contact

For any questions or further discussion about this work, feel free to contact me at stefanotonini26@gmail.com.
