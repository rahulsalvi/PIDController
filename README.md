# PIDController
Implements an PID Controller in C++

## Requirements
 - An implementation of the Timer object

## Brief Usage
 - Create a PIDController object, passing in a timer object to use and the set of coefficients
 - Call start() to begin
 - Call getOutput() with either a setpoint and input or an error
