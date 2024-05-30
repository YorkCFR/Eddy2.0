import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
import http.server
import socketserver

# hack
class Handler(http.server.SimpleHTTPRequestHandler):
    ROOT= "."
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=Handler.ROOT, **kwargs)


class WebUI(Node):
    def __init__(self):
        super().__init__('eddy_webui')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('root', "/home/jenkin/Documents/Eddy2.0/ros2_ws/src/eddy_webui/www/")
        root = self.get_parameter('root').get_parameter_value().string_value

        self.declare_parameter('port', 8081)
        port = self.get_parameter('port').get_parameter_value().integer_value

        Handler.ROOT = root
        with socketserver.TCPServer(("", port), Handler) as httpd:
            self.get_logger().info(f'{self.get_name()} serving')
            httpd.serve_forever()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = WebUI()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

