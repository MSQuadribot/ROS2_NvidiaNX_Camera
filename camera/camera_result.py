import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from std_msgs.msg import UInt8MultiArray

shape = [540,960,3]

resolution = shape[0]*shape[1]*shape[2]

class CameraResult(Node):

    def __init__(self):
        '''
        This function is called when the node is created.
        The node will subscribe to the topic "camera_data".
        Using the retreived images, the node will display images on the screen.
        For that purpose, it will use OpenCV.
        '''

        super().__init__('Camera_result')
        self.subscription = self.create_subscription(UInt8MultiArray,'camera_data', self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.img = None

    def listener_callback(self, msg): 
        '''
        As the node subscribe to the topic, the callback is used.
        The 1D array received from the topic is converted to a 3D array using numpy.
        Then, the image is displayed on the screen.
        It is important to note that the received Image is in RGB format while openCV needs BGR.
        '''          

        if msg.data == []:
            self.get_logger().info('Subscribing: Unsuccessful')
        else :
            self.get_logger().info('Subscribing: Successful')
            self.img = cv2.cvtColor(np.reshape(np.array(msg.data), (2*shape[0], shape[1], shape[2])), cv2.COLOR_BGR2RGB)


def main(args=None):
    '''
    This function is called when the user launch the executable file.
    Ros2 will directly start the program here, according to the setup file.
    The loop will use the callback to display the image on the screen.
    '''

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