import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from eddy_interfaces.msg import ThrusterStatus
from std_msgs.msg import String, Header
import serial

class ThrusterNode(Node):
    def __init__(self):
        super().__init__('thruster_controller')
        self.declare_parameter('thruster', 'thruster')
        thruster = self.get_parameter('thruster').get_parameter_value().string_value
        self.declare_parameter('port', '/dev/ttyUSB0')
        port = self.get_parameter('port').get_parameter_value().string_value

        self._publisher = self.create_publisher(ThrusterStatus, thruster, QoSProfile(depth=1))

        try:
            self.get_logger().info(f'{self.get_name()} connecting to serial controller on {port}')
            self._ser = serial.Serial(port, 9600, timeout=0.01)
            self.get_logger().info(f'{self.get_name()} connected to {self._ser.name}')
        except serial.SerialException as e:
            self.get_logger().error(f'{self.get_name()} unable to connect to port {port} {e}')
            exit(0)

        self._line = ""
        self._timer = self.create_timer(0.25, self._callback)
        self.get_logger().info(f'{self.get_name()} Waiting for thruster driver to wake up (approx 7 seconds)')

    def _callback(self):
        while True:
            avail = self._ser.readline()
            if len(avail) == 0:
                break
            q = bytes(x for x in avail if x < 127)
            self._line = self._line + q.decode("utf-8")

        pos = self._line.find("\r\n")
        if pos == -1:
            return
        message = self._line[:pos-1]
        self._line = self._line[pos+2:]

        while True:
            pos = self._line.find("\r\n")
            if pos == -1:
                break
            message = self._line[:pos-1]
            self._line = self._line[pos+2:]

        message = message.split(", ")

        msg = ThrusterStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.version = message[0]
        msg.state = message[1]
        msg.man_port = float(message[2])
        msg.man_starboard = float(message[3])
        msg.ros_port = float(message[4])
        msg.ros_starboard = float(message[5])
        msg.ros_light = float(message[6])
        self._publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = ThrusterNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
