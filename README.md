# Stepper-Motor-Microstepping
This contains code for operating a stepper motor for microstepping using a motor drive and 24V-6A power source.

#### Sub repository Arduino has code for operating stepper motor using arduino and it prints data on serial monitor of arduino IDE.
#### Sub repository scripts contains Arduino code and python code using pyserial for controlling stepper through linux shell using UART comunication.  
#### Sub repository stepper_driver_ros_control_microstepping is a ROS package for driving stepper motor NEMA 23 and it has a python node using pyserial library which publishes the motor status on a ROS topic and a subscribes on the /interrupt ROS topic. The package offers functionality of software interrupt and can be accessed by publishing on /interrupt topic:
##### 1) publish 'a' to stop the motor
##### 2) publish 'b' to start the motor again
##### 3) publish 'c' to reset the arduino
