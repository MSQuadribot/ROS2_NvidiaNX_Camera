import rclpy
from rclpy.node import Node

import os
import time
import array
import requests

from PIL import Image as Img
from std_msgs.msg import UInt8MultiArray

shape = [360,640,3]
resolution = shape[0]*shape[1]*shape[2]

class CameraWebposting(Node):

    def __init__(self):
        super().__init__('Camera_webposting')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'camera_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.img = []

    def listener_callback(self, msg):
        start = time.time()                                                                                                                                                                                                                                                                                                                                                   
        if msg.data == []:
            self.get_logger().info('Subscribing Cam: Unsuccessful')
        else :
            self.get_logger().info('Subscribing Cam: Successful')

            if msg.data != []:
                data = Img.frombytes("RGB", (640,720),msg.data.tobytes())
                data.save('/home/qb/Desktop/dev_ws/src/camera/camera/Image.jpg')
            else:
                print('Error while processing')

            if os.path.isfile('/home/qb/Desktop/dev_ws/src/camera/camera/Image.jpg'):

                res = requests.post('https://scargo.fr',files = {'Image': open('/home/qb/Desktop/dev_ws/src/camera/camera/Image.jpg','rb')})

                self.get_logger().info(f'Response Code : {res.status_code}')

            print("--- %s seconds ---" % (time.time() - start))


def main(args=None):

    # make sure that there is np image in the folder at the start
    if os.path.isfile('/home/qb/Desktop/dev_ws/src/camera/camera/Image.jpg'):
        os.remove('/home/qb/Desktop/dev_ws/src/camera/camera/Image.jpg')
    else :
        print('No such file in directory')

    rclpy.init(args=args)

    Camera_webposting = CameraWebposting()

    rclpy.spin(Camera_webposting)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Camera_webposting.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()