import rclpy
from rclpy.node import Node

import os
import time
import array
import asyncio
import httpx


from PIL import Image as Img
from std_msgs.msg import UInt8MultiArray

shape = [360,640,3]
resolution = shape[0]*shape[1]*shape[2]

class CameraAsyncWebposting(Node):

    def __init__(self):
        super().__init__('Camera_asyncwebposting')
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'camera_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.loop = asyncio.get_event_loop()
        self.i = 1

    def listener_callback(self, msg):

        if msg.data == []:
            self.get_logger().info('Subscribing Cam: Unsuccessful')
        else :
            if msg.data != []:
                data = Img.frombytes("RGB", (640,720),msg.data.tobytes())
                data.save(f'/home/qb/Desktop/dev_ws/src/camera/camera/Image/Image{self.i}.jpg')
                if self.i < 25 : 
                    self.i += 1
                else :
                    self.i = 1
                    
            else:
                print('Error while processing')


def main(args=None):

    # make sure that there is np image in the folder at the start
    if os.path.isfile('/home/qb/Desktop/dev_ws/src/camera/camera/Image.jpg'):
        os.remove('/home/qb/Desktop/dev_ws/src/camera/camera/Image.jpg')
    else :
        print('No such file in directory')

    rclpy.init(args=args)

    Camera_asyncwebposting = CameraAsyncWebposting()

    while True:
        start = time.time()
        rclpy.spin_once(Camera_asyncwebposting)
        print("--- %s seconds ---" % (time.time() - start))

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Camera_asyncwebposting.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()