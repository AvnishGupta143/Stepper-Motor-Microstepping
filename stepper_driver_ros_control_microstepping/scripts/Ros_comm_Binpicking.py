#!/usr/bin/env python

import serial
import time
import os
import rospy
import std_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Int32
import serial.tools.list_ports as port

class serial_communication(object):

	def __init__(self):               
		super(serial_communication, self).__init__()
		
		self.ser = serial.Serial()

		#Finding the port at which we can connect
		portlist = list(port.comports())
		for p in portlist:
			if "/dev/ttyACM" in str(p):
				self.arduino_comport = str(p).split(" ")				
		self.ser.port = self.arduino_comport[0]

		self.ser.baudrate = 57600
		self.ser.bytesize = serial.EIGHTBITS
		self.ser.parity = serial.PARITY_NONE
		self.ser.timeout = None       # block read 
		self.ser.xonxoff = False      # disable software flow control
		self.ser.rtscts = False       # disable hardware (RTS/CTS) flow control
		self.ser.dsrdtr = False       # disable hardware (DSR/DTR) flow control
		self.ser.writeTimeout = None  # timeout for write
		self.steps_per_revolution = str()
		self.steps_to_target = str()
		self.step_interval_micro = str()
		self.theoritical_time_taken = float()
		self.speed = str()
		self.moving_fwd_time = float()
		self.moving_back_time = float()
		rospy.init_node('ros_communication', anonymous=True)
		self.motor_pub = rospy.Publisher('/Motor_status', std_msgs.msg.String, queue_size = 1000)
		self.interrupt_status = str();

	def callback(self,msg_interrupt):
		# a to stop the motor
		# b to start the motor
		# c to reset the controller 
		if (msg_interrupt.data == 'a'):
			self.ser.write("a".encode())
			print("--------Motor Stopped-------")
			self.interrupt_status = msg_interrupt.data
		if (msg_interrupt.data == 'b'):
			self.ser.write("b".encode())
			print("---------Taking scan------")
			self.interrupt_status = msg_interrupt.data
		if (msg_interrupt.data == 'c'):
			self.ser.write("c".encode())
			print("--------Controller Resetted!!!Restart your Program-------")
			self.interrupt_status = msg_interrupt.data

	def start(self):
		try:
			self.ser.open()
			if(self.ser.isOpen()): 
				rospy.loginfo("Communication started successfully at %s",self.ser.portstr)
		except:
			rospy.loginfo("Failed to connect at any Port !!!!....Please check the connection")
			exit()

	def set_speed(self):
		print("Enter the speed in mm/s: ")
		self.speed = str(raw_input())

		# This is what speed.encode() does
		byte_speed = bytes(self.speed)
		
		self.ser.write(byte_speed)
		rospy.loginfo("Recieved speed as %s in mm/s", self.speed)
		time.sleep(0.3)

	def select_stepping_mode(self):
		
		print("Press '0' to select 200 steps/rev")
		print("Press '1' to select 400 steps/rev")
		print("Press '2' to select 800 steps/rev")
		print("Press '3' to select 1600 steps/rev")
		print("Press '4' to select 3200 steps/rev")
		print("Press '5' to select 6400 steps/rev")
		print("Press '6' to select 10000 steps/rev")
		print("Press '7' to select 12800 steps/rev")
		print("Press '8' to select 16000 steps/rev")
		print("Press '9' to select 20000 steps/rev")

		print("Enter the option: ")
		option = str(input())
		#using the encode function the data is encoded in form of bytearray and on serial we can only write data in bytes
		self.ser.write(option.encode())
		rospy.loginfo("Recieved the stepping mode as %s !!",option)
		time.sleep(0.3)

	def calculations(self):
		# ser.readline() will read untill it encounters a '\n' character and then terminates reading the '\n' character
		# Also the data read is in the form of byte array and hence decode() method is used to convert byte array to string
		self.theoritical_time_taken = float(self.ser.readline().decode().strip('\r\n'))/100
		self.steps_per_revolution = self.ser.readline().decode().strip('\r\n')
		self.steps_to_target = self.ser.readline().decode().strip('\r\n')
		self.step_interval_micro = self.ser.readline().decode().strip('\r\n')
		rospy.set_param('theoritical_time_taken',self.theoritical_time_taken)
		rospy.set_param('steps_to_target',self.steps_to_target)
		rospy.set_param('step_interval_micro',self.step_interval_micro)

	def close(self):
		self.ser.close()

	def reset_buffer(self):
		self.ser.reset_input_buffer()
		self.ser.reset_output_buffer()
		time.sleep(0.3)

	def start_the_motor(self):
		os.system("clear")		
		print("Stepping Mode: ", int(self.steps_per_revolution) ," Steps/rev")
		print("Step Interval in Microseconds: ", int(self.step_interval_micro))
		print("Total Steps to reach the target: ", int(self.steps_to_target))
		print("speed in mm/s:", float(self.speed))
		print("Theoritical Time Taken: " + str(self.theoritical_time_taken) + " Seconds")

		print(" Press Enter to Start the motor: ")
		raw_input()
		
		self.interrupt_sub = rospy.Subscriber('/Interrupt', std_msgs.msg.String, self.callback,queue_size = 5)
		print "publish 'a' on the /Interrupt topic: to stop the motor\npublish 'b' on the /Interrupt topic: to start the motor again\npublish 'c' on the /Interrupt topic: to reset the controller" 
		self.ser.write("1".encode())		
		print("------------------Motor Started-----------")	

	def motor_move_fwd(self):
		self.moving_fwd_time = float(self.ser.readline().decode().strip('\r\n'))/100
		print("Actual time taken to move Forward:",self.moving_fwd_time,"Seconds")
		s = "Actual time taken to move Forward: " + str(self.moving_fwd_time) + " Seconds"
		self.motor_pub.publish(s)
		self.ser.reset_input_buffer()

	def motor_move_back(self):	
		self.moving_back_time = float(self.ser.readline().decode().strip('\r\n'))/100
		print("Actual time taken to move Backward:",self.moving_back_time,"Seconds")
		s = "Actual time taken to move Backward: " + str(self.moving_back_time) + " Seconds"
		self.motor_pub.publish(s)
		self.ser.reset_input_buffer()

if __name__ == '__main__':
	#while port is opened
	Port = serial_communication()
	Port.start()
	Port.set_speed()
	Port.reset_buffer()
	Port.select_stepping_mode()
	Port.calculations()
	Port.reset_buffer()
	Port.start_the_motor()
	Port.reset_buffer()

	instruction = True
	while(Port.ser.isOpen()):

		if instruction:
			print ' Waiting for command to take scan'
			instruction = False

		if (Port.interrupt_status == 'b'):
			Port.motor_move_fwd()
			Port.motor_move_back()
			print 'Took one scan' 
			# here we set the Port.interrupt status='a' so that it stops after one roundtrip and similarly we will be making flag = 1 in arduino code after ececution of one round
			Port.interrupt_status = 'a'
			instruction = True

	Port.close()
