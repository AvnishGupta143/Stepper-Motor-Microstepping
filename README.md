# Stepper-Motor-Microstepping
This contains code for operating a stepper motor for microstepping using a motor drive and 24V-6A power source.

#### Sub repository Arduino has code for operating stepper motor using arduino and it prints data on serial monitor of arduino IDE.
#### Sub repository scripts contains Arduino code and python code using pyserial for controlling stepper through linux shell using UART comunication.  
#### Sub repository Stepper_driver_ROS is a ROS package for driving stepper motor NEMA 23 and it has a python node which publishes the motor status on a topic and a subscribes on the interupt topic for stopping the motor using software interrupt and the arduino library for applying software interrupt.
