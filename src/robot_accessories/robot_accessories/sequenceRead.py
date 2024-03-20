###################################################
###												###
###			Noeud de lecture de sequence		###
###				CRUBS - Lorient					###
###	  		club.robotique.ubs@gmail.com		###
###	   			file creation 07/02/2024		###
###			last modification 07/02/2024		###
###												###
###################################################

## Import
import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Int16MultiArray, Bool
from geometry_msgs.msg import Twist

class sequenceReader(Node):
	def __init__(self):
		super().__init__('sequenceReaderNode')
		
		# ROS2 publishers
		self.publisherCmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
		self.publisherServos = self.create_publisher(Int16MultiArray, '/servos_cmd', 10)
		self.publisherSolar = self.create_publisher(Bool, '/solar_wheel_state', 10)
		
		# Timer init
		self.timerTimer = self.create_timer(0.1, self.timer_timer_callback)
		
		# File containing the sequence
		self.sequenceFile = '/home/crubs/Documents/dev_ws/src/robot_accessories/robot_accessories/sequence.txt'
		self.sequenceLines = []
		self.sequencePos = 0
		try:
			self.sequenceData = open(self.sequenceFile, 'r')
			self.sequenceLines = self.sequenceData.readlines()
		except: print("File not found")
		
	
	def timer_timer_callback(self):
		"""
		# Read line from the sequence file
		if self.sequenceData is EOFError:
			line = [0.0,0.0,0,0,0,0]
		else:
			line  = self.sequenceData.readline().split()
		"""

		line = self.sequenceLines[self.sequencePos].split()
		self.sequencePos += 1

		vx = line[0]
		vrz = line[1]
		servosCmd = [int(line[2]), int(line[3]), int(line[4])]
		#solarWheel = bool(int(line[5]))
		if line[5] == "1":
			solarWheel = True
		else:
			solarWheel = False
		
		# Generate the message to be sent over /cmd_vel
		msgCmdVel = Twist()
		msgCmdVel.linear.x = float(vx)
		msgCmdVel.linear.y = 0.0
		msgCmdVel.linear.y = 0.0
		msgCmdVel.angular.x = 0.0
		msgCmdVel.angular.y = 0.0
		msgCmdVel.angular.z = float(vrz)

		# Generate the message to be sent over /servos_cmd
		msgServos = Int16MultiArray()
		msgServos.data = servosCmd
		
		# Generate the message to be sent over /solar_wheel_state
		msgSolar = Bool()
		msgSolar.data = solarWheel

		# Publishing the messages to the ros topics
		self.publisherCmdVel.publish(msgCmdVel)
		self.publisherServos.publish(msgServos)
		self.publisherSolar.publish(msgSolar)


def main(args=None):
	rclpy.init(args=args)
	node = sequenceReader()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
