import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray

shape = [360,640,3]
resolution = shape[0]*shape[1]*shape[2]

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('Camera_subscriber')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'camera_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):                                                                                                                                                                                                                                                                                                                                                   
        if msg.data == []:
            self.get_logger().info('Subscribing: "%s"' % msg.data)
        else :
            self.get_logger().info('Subscribing: "%s" ... "%s" ... "%s"' % (msg.data[0:3], msg.data[resolution-1:resolution],msg.data[resolution-4:resolution-1]))


def main(args=None):
    rclpy.init(args=args)

    Camera_subscriber = CameraSubscriber()

    rclpy.spin(Camera_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()