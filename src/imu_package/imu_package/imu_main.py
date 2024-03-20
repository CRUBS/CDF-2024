import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from icm20948 import ICM20948
import numpy as np
from tf_transformations import euler_from_quaternion
from .submodules.madgwick import Madgwick


class NodeImu(Node):
	"""
	__init__ initialises the global processes and variables
	"""

	def __init__(self):
		super().__init__('Imu_readings')
		self.period = 0.04  # Period between callbacks
		# Create the publisher of an IMU message on the node Imu_readings
		#self.publisher_ = self.create_publisher(Imu, 'imu_readings', 10)
		self.subscribtion_ = self.create_subscription(Imu, 'imu', self.callback, 10) 
		
		# Launches the function timer_callbacks every period
		self.timer_ = self.create_timer(self.period,  self.timer_callbacks)
		self.get_logger().info('Node initialized')
		#self.sensor = ICM20948()  # Initialise the IMU ICM20948
		self.madgwick = Madgwick()  # Initialise a Madgwick filter
		self.Q = np.array([1.0, 0.0, 0.0, 0.0])  # Initialise a quaternion
		self.imu = Imu()

	"""
	ned_to_enu takes a list in the NED (North, East, Down) format and switch it to the ENU (East, North, Up) format
	@param ned a list of coordinates in the NED format
	@return enu a list of coordinates in the ENU format
	"""

	@staticmethod
	def ned_to_enu(ned) -> tuple:
		return ned[1], ned[0], -ned[2]

	"""
	imu_treatement gets the data from the IMU, traet it and returns it in the IMU format
	@return imu the IMU message
	"""

	def imu_treatement(self):
	
		# acc_measurement  = self.ned_to_enu( acc_gyro_measurement[0:3] ) 			#acc in g
		# gyro_measurement = self.ned_to_enu( acc_gyro_measurement[3:] )  			#gyro in degree per second
		# mag_measurement  = self.ned_to_enu( self.sensor.read_magnetometer_data() ) #mag in microtesla

		acc_measurement = self.assign_2_array(self.imu.linear_acceleration)  # acc in g
		gyro_measurement = self.assign_2_array(self.imu.angular_velocity)  # gyro in degree per second
		
		acc_measurement = [acc_measurement[0] * 9.81, acc_measurement[1] * 9.81,
						acc_measurement[2] * 9.81]  # acc from g to m/(s*s)
		gyro_measurement = [gyro_measurement[0] * np.pi / 180, gyro_measurement[1] * np.pi / 180,
							gyro_measurement[2] * np.pi / 180]  # gyro from dps to radian/second
		
		# Get time
		self.imu.header.stamp = self.get_clock().now().to_msg()

		# Assign the imu data to a vect3 object

		vec3_acc = self.assign_2_vect(acc_measurement)
		vec3_gyro = self.assign_2_vect(gyro_measurement)

		# Determine the quaternarion

		#self.Q = self.madgwick.madgwick_ahrs_update(gyro_measurement[0], gyro_measurement[1], gyro_measurement[2],
		#											acc_measurement[0], acc_measurement[1], acc_measurement[2],
		#											1 / self.period)

		# Checks the variables

		# self.get_logger().info(f"Received acceleration: {vec3_acc}")
		# self.get_logger().info(f"Received gyro: {vec3_gyro}")
		# self.get_logger().info(f"Received mag: {mag_measurement}\n")
		# self.get_logger().info(f"Received quat: {self.Q}\n")
		# self.get_logger().info(f"Received euler: {self.quat_2_euler(self.assign_2_quat())}\n")

		# Assign the value to imu

		self.imu.linear_acceleration = vec3_acc
		self.imu.linear_acceleration_covariance = [5.548E-4, 0.0, 0.0,
											0.0, 5.065E-4, 0.0,
											0.0, 0.0, 5.804E-4]

		self.imu.angular_velocity = vec3_gyro
		self.imu.angular_velocity_covariance = [4.5598E-6, 0., 0.,
										0., 4.1822E-6, 0.,
										0., 0., 4.4396E-6]

		self.imu.orientation = self.assign_2_quat()

		# As the first term of the matrix is -1 the quaternion is currently ignored
		self.imu.orientation_covariance = [-1., 0., 0.,
									0., 0., 0.,
									0., 0., 0.]

		return self.imu

	"""
	quat_2_euler takes a quaternion and returns a Vector3 with the euler angles
	@param array3 a numpy array of size 3
	@return vect3 an object of type Vector3
	"""

	@staticmethod
	def quat_2_euler(quat: Quaternion) -> Vector3:
		euler = Vector3()
		# Euler angles (yaw, pitch and roll)
		euler.x, euler.y, euler.z = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
		return euler

	"""
	assign_2_vect takes a numpy array and returns a Vector3
	@param array3 a numpy array of size 3
	@return vect3 an object of type Vector3
	"""

	@staticmethod
	def assign_2_vect(array3: np.array) -> Vector3:
		vect3 = Vector3()
		vect3.x, vect3.y, vect3.z = array3[0], array3[1], array3[2]

		return vect3

	"""
	assign_2_array takes a numpy array and returns a Vector3
	@param vect3 an object of type Vector3
	@return  array3 a numpy array of size 3
	"""

	@staticmethod
	def assign_2_array(vect3: Vector3) -> np.array:
		array3 = [0,0,0]
		array3[0], array3[1], array3[2] = vect3.x, vect3.y, vect3.z

		return array3

	"""
	assign_2_quat assign Q to a Quaternion
	@return quat an object of type Quaternion
	"""

	def assign_2_quat(self) -> Quaternion:
		quat = Quaternion()
		quat.x, quat.y, quat.z, quat.w = self.Q[0], self.Q[1], self.Q[2], self.Q[3]

		return quat

	"""
	timer_callbacks takes the imu data and publishes it on the node Imu_readings
	"""

	#def timer_callbacks(self):
		#self.publisher_.publish(self.imu_treatement())

	"""
	callback takes the Odometry message and extract the position and euler angles from it
	"""

	def callback(self, msg):
		self.imu = msg


def main(args=None):
	rclpy.init(args=args)
	node = NodeImu()
	rclpy.spin(node)
	rclpy.shutdown()


if __name__ == '__main__':
	main()
