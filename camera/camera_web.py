import rclpy
from rclpy.node import Node

import os
import time
import requests
from datetime import datetime

from PIL import Image as Img
from std_msgs.msg import UInt8MultiArray

class CameraWebposting(Node):

    def __init__(self):
        '''
        This function is called when the node is created.
        The node will subscribe to the topic "camera_data".
        With the retrieved data, images will be posted to the server.
        '''

        super().__init__('Camera_webposting')
        self.subscription = self.create_subscription(UInt8MultiArray,'camera_data',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.URL = 'https://rbox.live/image.php'
        self.i = 1

    def listener_callback(self, msg):
        '''
        This callback is called by the node everytime it has to publish a new image.
        The image is retrieved from the message and posted to the server.
        This process is repeated everytime a new image is published by "camera_data".
        It can take some time, depending on the quality of the internet connection. (~1 second)
        '''

        start = time.time()                                                                                                                                                                                                                                                                                                                                                   
        if msg.data == [] or self.i > 10:
            self.get_logger().info('Subscribing Cam: Unsuccessful')
        else :
            self.get_logger().info('Subscribing Cam: Successful')

            if msg.data != []:
                data = Img.frombytes("RGB", (960,1080),msg.data.tobytes())
                data.save(f'/home/qb/Desktop/dev_ws/src/camera/camera/Image{self.i}.jpg')
            else:
                print('Error while processing')

            if os.path.isfile(f'/home/qb/Desktop/dev_ws/src/camera/camera/Image{self.i}.jpg'):

                res = requests.post(self.URL,files = {'tmp_name':f"Image{self.i}.jpg",'image': open(f'/home/qb/Desktop/dev_ws/src/camera/camera/Image{self.i}.jpg','rb'),'date':datetime.today().strftime('%d-%m-%Y %H:%M')})

                self.get_logger().info(f'Response Code : {res.status_code}')
                self.get_logger().info(f'Image counter : {self.i}')
            
            if self.i < 10:
                self.i +=1
            else:
                self.i = 1

            print("--- %s seconds ---" % (time.time() - start))


def main(args=None):
    '''
    This function is called when the user launch the executable file.
    Ros2 will directly start the program here, according to the setup file.
    '''

    # make sure that there is np image in the folder at the start
    for i in range(1,11):
        if os.path.isfile(f'/home/qb/Desktop/dev_ws/src/camera/camera/Image{i}.jpg'):
            os.remove(f'/home/qb/Desktop/dev_ws/src/camera/camera/Image{i}.jpg')

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