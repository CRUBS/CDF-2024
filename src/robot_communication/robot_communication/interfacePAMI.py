import rclpy
import socket
import time
from rclpy.node import Node
from std_msgs.msg import Int16

class PAMISubscriber(Node):
	def __init__(self):
		super().__init__('pami_subscriber')
		self.envoye = False
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.socket.bind((host,port))
		self.get_logger().info('Communication PAMI démarrée')
		self.socket.listen(1)
		self.esp1_conn, self.esp1_address = self.socket.accept()
		self.get_logger().info('Un connexion a réussi')
		self.get_logger().info('Adresse : "%s"' % self.esp1_address[0])
		self.get_logger().info('Adresse : "%s" ; Ping' % self.esp1_address[0])

		data = "ping"
		data = data.encode('utf-8')
		self.esp1_conn.send(data)
		data = self.esp1_conn.recv(1024)
		data = data.decode("utf-8")
		if data == "pingReturn":
			self.get_logger().info('Adresse : "%s" ; Ping => OK' % self.esp1_address[0])

		time.sleep(2)
		self.subscription = self.create_subscription(
			Int16,
			'/timer',
			self.listener_callback,
		10)
		self.subscription  # prevent unused variable warning

	def listener_callback(self, msg):
		self.get_logger().info('Timer : "%d"' % msg.data)
		if msg.data > 20 and self.envoye == False:
			data = "startSequence"
			data = data.encode('utf-8')
			self.esp1_conn.send(data)
			self.get_logger().info('Adresse : "%s" ; Ordre de demarrage envoye' % self.esp1_address[0])
			data = self.esp1_conn.recv(1024)
			data = data.decode('utf-8')
			if data == "startSequenceReturn":
				self.get_logger().info('Adresse : "%s" ; Ordre de lancement confirme' % self.esp1_address[0])
			self.envoye = True

host, port = ('', 5566)

def main(args=None):
	rclpy.init(args=args)
	pami_subscriber = PAMISubscriber()
	rclpy.spin(pami_subscriber)
	esp1_conn.close()
	socket.close()
	pami_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
