import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from std_msgs.msg import UInt8MultiArray

shape = [540,960,3]

resolution = shape[0]*shape[1]*shape[2]

class CameraResult(Node):

    def __init__(self):
        super().__init__('Camera_result')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'camera_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.img = None

    def listener_callback(self, msg):                                                                                                                                                                                                                                                                                                                                                   
        if msg.data == []:
            self.get_logger().info('Subscribing: Unsuccessful')
        else :
            self.get_logger().info('Subscribing: Successful')
            self.img = cv2.cvtColor(np.reshape(np.array(msg.data), (2*shape[0], shape[1], shape[2])), cv2.COLOR_BGR2RGB)


def main(args=None):
    rclpy.init(args=args)

    Camera_result = CameraResult()

    while True :
        
        rclpy.spin_once(Camera_result)
        cv2.imshow('Frame', Camera_result.img)
        if  cv2.waitKey(1) == ord('q'):
            break


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Camera_result.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()