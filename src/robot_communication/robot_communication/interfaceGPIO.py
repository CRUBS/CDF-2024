###################################################
###												###
###	  Interface GPIO robot 2023-2024			###
###		CRUBS - Lorient							###
###	  club.robotique.ubs@gmail.com				###
###	   file creation 24/03/2024					###
###	last modification --/--/2024				###
###												###
###################################################

# Import libs
import time
import RPi.GPIO as GPIO

# Import libs ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

GPIO.setmode(GPIO.BOARD) #mode num de pin

motorsResetPins = [11, 13, 15]
teamPin = 18
starterPin = 16

GPIO.setup(motorsResetPins[0], GPIO.OUT, initial=GPIO.HIGH) #reset m1
GPIO.setup(motorsResetPins[0], GPIO.OUT, initial=GPIO.HIGH) #reset m2
GPIO.setup(motorsResetPins[0], GPIO.OUT, initial=GPIO.HIGH) #reset m3
GPIO.setup(starterPin, GPIO.IN) #input starter
GPIO.setup(teamPin, GPIO.IN) #input color team

class gpioInterface(Node):
	def __init__(self):
		super().__init__('interfaceGPIO')
		#ros2 timers
		self.timerTeam = self.create_timer(1, self.timer_team_callback)
		self.timerStarter = self.create_timer(0.25, self.timer_starter_callback)
	
	    #ros2 publisher
		self.publisherTeam = self.create_publisher(String, '/Team', 10)
		self.publisherStarter = self.create_publisher(Bool, '/StarterRemoved', 10)
		
	    #ros2 subscription
		self.subscriptionMotorState = self.create_subscription(String, '/timers_status/motors', self.callback_motors_state, 10)
	
	def timer_team_callback(self):
		msg = String()
		if GPIO.input(teamPin):
			msg.data = 'Bleu'
		else :
			msg.data = 'Jaune'
		self.publisherTeam.publish(msg)

	def timer_team_callback(self):
		msg = Bool()
		if GPIO.input(starterPin):
			msg.data = False
		else :
			msg.data = True
		self.publisherStarter.publish(msg)

	def callback_motors_state(self, msg):
		if msg.data == 'error':
			GPIO.output(motorsResetPins[0], GPIO.LOW)
			GPIO.output(motorsResetPins[1], GPIO.LOW)
			GPIO.output(motorsResetPins[2], GPIO.LOW)
			time.sleep(0.5)
			GPIO.output(motorsResetPins[0], GPIO.HIGH)
			GPIO.output(motorsResetPins[1], GPIO.HIGH)
			GPIO.output(motorsResetPins[2], GPIO.HIGH)
		else:
			pass

        

def main(args=None):
	rclpy.init(args=args)
	node = gpioInterface()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
		
