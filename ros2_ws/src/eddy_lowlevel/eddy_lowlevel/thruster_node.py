#
# This encapsulates low level thruster control (wrench->thruster speed). This also includes support
# for more pass-through information (light, etc) 
#
# Version 2.0
#       Now with subscribers
# Version 1.0
#	basic functionality
#

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from eddy_interfaces.msg import ThrusterStatus
from geometry_msgs.msg import Wrench, Vector3
from std_msgs.msg import String, Header, Float64
import serial

class ThrusterNode(Node):
    KGF_N = 9.80665

#   These are thrust values for values betwen -1 and +1 for ESC setting from 1100-1900
    T200_THRUST_VALUES = [-3.52, -3.50, -3.49, -3.45, -3.40, -3.36, -3.29, -3.25, -3.19, -3.14, -3.10, 
                          -3.06, -3.00, -2.94, -2.88, -2.85, -2.78, -2.76, -2.69, -2.64, -2.59, -2.53, 
                          -2.49, -2.45, -2.41, -2.35, -2.34, -2.26, -2.20, -2.18, -2.12, -2.05, -2.03, 
                          -1.99, -1.91, -1.89, -1.82, -1.76, -1.72, -1.68, -1.63, -1.58, -1.56, -1.52, 
                          -1.48, -1.44, -1.40, -1.37, -1.32, -1.28, -1.24, -1.19, -1.17, -1.12, -1.09, 
                          -1.05, -1.02, -0.98, -0.95, -0.92, -0.88, -0.85, -0.81, -0.77, -0.74, -0.70, 
                          -0.68, -0.65, -0.62, -0.59, -0.55, -0.52, -0.49, -0.46, -0.43, -0.40, -0.37, 
                          -0.35, -0.32, -0.29, -0.26, -0.24, -0.21, -0.19, -0.16, -0.15, -0.12, -0.10, 
                          -0.08, -0.07, -0.05, -0.03,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00, 
                           0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,  0.05, 
                           0.06,  0.08,  0.10,  0.12,  0.15,  0.18,  0.20,  0.23,  0.26,  0.29,  0.33, 
                           0.36,  0.39,  0.43,  0.46,  0.50,  0.53,  0.58,  0.62,  0.64,  0.69,  0.73, 
                           0.77,  0.83,  0.85,  0.89,  0.92,  0.97,  1.00,  1.05,  1.09,  1.14,  1.20, 
                           1.23,  1.28,  1.32,  1.37,  1.41,  1.46,  1.51,  1.55,  1.61,  1.65,  1.71, 
                           1.76,  1.81,  1.85,  1.91,  1.96,  2.00,  2.09,  2.12,  2.16,  2.25,  2.27, 
                           2.34,  2.43,  2.50,  2.56,  2.64,  2.66,  2.76,  2.78,  2.88,  2.93,  2.99, 
                           3.05,  3.13,  3.19,  3.23,  3.32,  3.36,  3.42,  3.49,  3.57,  3.62,  3.69, 
                           3.77,  3.84,  3.92,  3.98,  4.03,  4.11,  4.15,  4.21,  4.30,  4.38,  4.42, 
                           4.51, 4.53, 4.52]
    NUM_THRUST_VALUES = len(T200_THRUST_VALUES)

    EDDY_THRUSTER_SEPARATION = 0.35
    EDDY_MAX_REVERSE_THRUST = KGF_N * T200_THRUST_VALUES[0]
    EDDY_MAX_FORWARD_THRUST = KGF_N * T200_THRUST_VALUES[-1]


    def __init__(self):
        super().__init__('thruster_controller')
        self.declare_parameter('motors', 'motor_setting')
        motors = self.get_parameter('motors').get_parameter_value().string_value
        self.declare_parameter('cmd_thrust', 'cmd_thrust')
        thrust = self.get_parameter('cmd_thrust').get_parameter_value().string_value
        self.create_subscription(Wrench, thrust, self._thrust_callback, QoSProfile(depth=1))
        self.declare_parameter('cmd_light', 'cmd_light')
        light = self.get_parameter('cmd_light').get_parameter_value().string_value
        self.create_subscription(Float64, light, self._light_callback, QoSProfile(depth=1))
        self.declare_parameter('port', '/dev/ttyUSB0')
        port = self.get_parameter('port').get_parameter_value().string_value
        self.declare_parameter('updaterate', 0.25)
        updateRate = self.get_parameter('updaterate').get_parameter_value().double_value
      
        self._publisher = self.create_publisher(ThrusterStatus, motors, QoSProfile(depth=1))

        try:
            self.get_logger().info(f'{self.get_name()} connecting to serial controller on {port}')
            self._ser = serial.Serial(port, 9600, timeout=0.01)
            self.get_logger().info(f'{self.get_name()} connected to {self._ser.name}')
        except serial.SerialException as e:
            self.get_logger().error(f'{self.get_name()} unable to connect to port {port} {e}')
            exit(0)

        self._brightness = 0
        self._port_thrust = 0
        self._starboard_thrust = 0

        self._line = ""
        self._timer = self.create_timer(updateRate, self._timer_callback)
        self.get_logger().info(f'{self.get_name()} Waiting for thruster driver to wake up (approx 7 seconds)')

    def _mapf(val, imin, imax, omin, omax):
        return (((val)-(imin)) * ((omax)-(omin)) / ((imax)-(imin)) + (omin))

    def _t200_rate(i):
        """For an entry i in the T200_THRUST_VALUES table, obtain a proportion"""
        return -1.0 + 2.0 * i / (ThrusterNode.NUM_THRUST_VALUES - 1)

    def _t200_thrust(thrust) :
        """Map desired thrust (N) to a value between -1 and +1"""
        thrust_kgf = thrust / ThrusterNode.KGF_N
	
        # Perform a binary search for the desired thrust value
        left = 0 
        right = ThrusterNode.NUM_THRUST_VALUES[-1]
        while left <= right:
            i = int((left + right) / 2)
            if ThrusterNode.T200_THRUST_VALUES[i] < thrust_kgf:
                left = i + 1
            elif ThursterNode.T200_THRUST_VALUES[i] > thrust_kgf:
                right = i - 1
            else:
                return ThrusterNode._t200_rate(i)
	
        # Exact thrust not found in samples, interpolate to pwm value using closest samples  
        return ThrusterNode._mapf(thrust, ThrusterNode._T200_THRUST_VALUES[right], ThursterNode._T200_THRUST_VALUES[left], 
                               ThrusterNode._t200_rate(right), ThrusterNode._t200_rate(left))

    def _constraint(val, min_val, max_val):
        """bind val to bewteen min and max"""
        return min(max_val, min(min_val, val))

    def _update_hardware(self):
        port = int(1000 * self._port_thrust)
        starboard = int(1000 * self._starboard_thrust)
        brightness = int(1000 * self._brightness)
        buf = f'ED2{port: 5}{starboard: 5}{brightness: 5}\n'
        self._serial.write(buf.encode('utf-8'))

    def _light_callback(self, msg):
        """Process a request to change the lights"""
        self.get_logger().info(f'{self.get_name()} want to set the light to {msg}')
        self._brightness = ThrusterNode.constraint(msg.data, 0, 1)
        self._update_hardware()

    def _thrust_callback(self, msg):
        """Process a request to change the wrench on the vehicle"""
        self.get_logger().info(f'{self.get_name()} want to set the thrust to {msg}')

        fx = wrench.force.x;
        tauz = wrench.torque.z;
	
        # retrict torque to maximum capable by the robot (using reverse thrust)
        max_tauz = -2 * ThrusterNode.EDDY_MAX_REVERSE_THRUST * ThrusterNode.EDDY_THRUSTER_SEPARATION
        tauz = min(tauz, max_tauz)
        tauz = max(tauz, -max_tauz)
	
	# prioritize rotation
        left_thrust = -tauz / (2 * ThrusterNode.EDDY_THRUSTER_SEPARATION)
        right_thrust = tauz / (2 * ThrusterNode.EDDY_THRUSTER_SEPARATION)
	
	# scale back linear force to prevent oversaturating thrusters 
        max_fx = 0
        if tauz >= 0:
            if fx >= 0:
                max_fx = (ThrusterNode.EDDY_MAX_FORWARD_THRUST - right_thrust) * 2
                fx = min(max_fx, fx)
            else:
                max_fx = (ThrusterNode.EDDY_MAX_REVERSE_THRUST - left_thrust) * 2
                fx = max(max_fx, fx)
        else:
            if fx >= 0:
                max_fx = (ThrusterNode.EDDY_MAX_FORWARD_THRUST - left_thrust) * 2
                fx = min(max_fx, fx);
            else:
                max_fx = (ThrusterNode.EDDY_MAX_REVERSE_THRUST - right_thrust) * 2
                fx = max(max_fx, fx)
	
        left_thrust = left_thrust + fx/2.0
        right_thrust = right_thrust + fx/2.0
	
        left_thrust = ThrusterNode.constrain(left_thrust, ThrusterNode.EDDY_MAX_REVERSE_THRUST, ThrusterNode.EDDY_MAX_FORWARD_THRUST)
        right_thrust = ThrusterNode.constrain(right_thrust, ThrusterNode.EDDY_MAX_REVERSE_THRUST, ThrusterNode.EDDY_MAX_FORWARD_THRUST)
	
        self._port_thrust = ThrusterNode._t200_thrust(left_thrust)
        self._starboard_thrust = ThrusterNode._t200_thrust(right_thrust)
        self._update_hardware()

    def _timer_callback(self):
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
        self.get_logger().info(f'{self.get_name()} got message {message}')

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
