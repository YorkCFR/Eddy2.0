import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from threading import Thread, Lock
from time import sleep


from geometry_msgs.msg import PointStamped

class WebUI(Node):
    def __init__(self):
        super().__init__('eddy_webui')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('www_root', "~/www")

        self._www_root = self.get_parameter('www_root').get_parameter_value().string_value
        self._target_topic = self.get_parameter('www_root').get_parameter_value().string_value

        self._thread = Thread(target = self._server)
        self._lock = Lock()
        self._thread.start()
 

    def _server(self):
        while True:
            self.get_logger().info(f'{self.get_name()} sleeping')
            sleep(1)
            
        
        



def main(args=None):
    rclpy.init(args=args)
    node = WebUI()
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

