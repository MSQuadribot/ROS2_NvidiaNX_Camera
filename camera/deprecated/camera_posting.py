import rclpy
from rclpy.node import Node

import numpy as np
import os
import pycurl
import time

from PIL import Image as Img
from std_msgs.msg import UInt8MultiArray

shape = [360,640,3]
resolution = shape[0]*shape[1]*shape[2]

class CameraWebposting(Node):

    def __init__(self):
        super().__init__('Camera_webposting')
        self.subscription0 = self.create_subscription(
            UInt8MultiArray,
            'camera0_data',
            self.listener0_callback,
            10)
        self.subscription1 = self.create_subscription(
            UInt8MultiArray,
            'camera1_data',
            self.listener1_callback,
            10)
        self.subscription0  # prevent unused variable warning
        self.subscription1  # prevent unused variable warning
        self.img0 = []
        self.img1 = []

    def listener0_callback(self, msg):                                                                                                                                                                                                                                                                                                                                                   
        if msg.data == []:
            self.get_logger().info('Subscribing Cam0: Unsuccessful')
        else :
            self.get_logger().info('Subscribing Cam0: Successful')
            self.img0 = np.array(msg.data)

    def listener1_callback(self, msg):                                                                                                                                                                                                                                                                                                                                                   
        if msg.data == []:
            self.get_logger().info('Subscribing Cam1: Unsuccessful')
        else :
            self.get_logger().info('Subscribing Cam1: Successful')
            self.img1 = np.array(msg.data)

    def process(self):
        if self.img0 != [] and self.img1 != []:
            self.img0 = np.reshape(self.img0, (shape[0], shape[1], shape[2]))
            self.img1 = np.reshape(self.img1, (shape[0], shape[1], shape[2]))
            data = Img.fromarray(np.hstack((self.img1 , self.img0)))
            data.save('/home/qb/Desktop/dev_ws/src/camera/camera/Image.png')
        else:
            return('Error while processing')


def main(args=None):

    # make sure that there is np image in the folder at the start
    if os.path.isfile('/home/qb/Desktop/dev_ws/src/camera/camera/Image.png'):
        os.remove('/home/qb/Desktop/dev_ws/src/camera/camera/Image.png')
    else :
        print('No such file in directory')

    rclpy.init(args=args)

    Camera_webposting = CameraWebposting()

    while True:
        
        start = time.time()

        rclpy.spin_once(Camera_webposting)
        rclpy.spin_once(Camera_webposting)
        rclpy.spin_once(Camera_webposting)
        rclpy.spin_once(Camera_webposting)
        rclpy.spin_once(Camera_webposting)
        Camera_webposting.process()

        if os.path.isfile('/home/qb/Desktop/dev_ws/src/camera/camera/Image.png'):
            c = pycurl.Curl()

            c.setopt(c.URL,'https://scargo.fr')

            c.setopt(c.POST,1)

            c.setopt(c.HTTPPOST, [('Image',(c.FORM_FILE, '/home/qb/Desktop/dev_ws/src/camera/camera/Image.png'))])

            c.perform()

            print('Response : %d' % c.getinfo(c.RESPONSE_CODE))

            c.close()
        
        print("--- %s seconds ---" % (time.time() - start))

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Camera_webposting.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()