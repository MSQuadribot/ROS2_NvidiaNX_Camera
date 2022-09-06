import rclpy
from rclpy.node import Node

import numpy as np
import cv2
import time

from std_msgs.msg import UInt8MultiArray

shape = [540,960,3]

resolution = shape[0]*shape[1]*shape[2]

def obstacle_avoid(depth_map, depth_thresh, output):

    # Mask to segment regions with depth less than threshold
    mask = cv2.inRange(depth_map,10,depth_thresh)

    # Check if a significantly large obstacle is present and filter out smaller noisy regions
    if np.sum(mask)/255.0 > 0.01*mask.shape[0]*mask.shape[1]:

        # Contour detection 
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = sorted(contours, key=cv2.contourArea, reverse=True)

        # Check if detected contour is significantly large (to avoid multiple tiny regions)
        if cv2.contourArea(cnts[0]) > 0.01*mask.shape[0]*mask.shape[1]:

            x,y,w,h = cv2.boundingRect(cnts[0])

            # finding average depth of region represented by the largest contour 
            mask2 = np.zeros_like(mask)
            cv2.drawContours(mask2, cnts, 0, (255), -1)

            # Calculating the average depth of the object closer than the safe distance
            depth_mean, _ = cv2.meanStdDev(depth_map, mask=mask2)

            # Display warning text
            cv2.putText(output, "WARNING !", (x+5,y-40), 1, 2, (0,0,255), 2, 2)
            cv2.putText(output, "Object at", (x+5,y), 1, 2, (100,10,25), 2, 2)
            cv2.putText(output, "%.2f cm"%depth_mean, (x+5,y+40), 1, 2, (100,10,25), 2, 2)
            print(f"x : {x}; y : {y}, w = {w}, h = {h}")
            print(depth_mean)

    else:
        cv2.putText(output, "SAFE!", (100,100),1,3,(0,255,0),2,3)

        cv2.imshow('output',output)

class CameraDepth(Node):

    def __init__(self):
        '''
        This function is called when the node is created.
        The node will subscribe to the topic "camera_data".
        Using the retreived images, the node will display images on the screen.
        For that purpose, it will use OpenCV.
        '''

        super().__init__('Camera_depth')
        self.subscription = self.create_subscription(UInt8MultiArray,'camera_data', self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.img_0 = None
        self.img_1 = None
        self.out = None

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
            image = cv2.cvtColor(np.reshape(np.array(msg.data), (2*shape[0], shape[1], shape[2])), cv2.COLOR_BGR2RGB)
            self.img_0 = image[:540]
            self.img_1 = image[541:]
            self.out = self.img_0


def main(args=None):
    '''
    This function is called when the user launch the executable file.
    Ros2 will directly start the program here, according to the setup file.
    The loop will use the callback to display the image on the screen.
    '''

    rclpy.init(args=args)

    Camera_depth = CameraDepth()

    M = 25 *0.315
    min_depth = 50
    max_depth = 500
    depth_tresh = 100.0

    # Reading the mapping values for stereo image rectification
    cv_file = cv2.FileStorage("/home/qb/Desktop/dev_ws/src/camera/camera/calibration.xml", cv2.FILE_STORAGE_READ)
    Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
    Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
    Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
    Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
    cv_file.release()

    # Setting parameters for StereoSGBM algorithm
    minDisparity = 16
    numDisparities = 96
    blockSize = 7
    disp12MaxDiff= 100
    uniquenessRatio = 1
    speckleWindowSize = 255
    speckleRange = 255


    # Creating an object of StereoBM algorithm
    stereo = cv2.StereoSGBM_create(minDisparity = minDisparity,
        numDisparities = numDisparities,
        blockSize = blockSize,
        disp12MaxDiff = disp12MaxDiff,
        uniquenessRatio = uniquenessRatio,
        speckleWindowSize = speckleWindowSize,
        speckleRange = speckleRange
    )
 

    while True :
        
        start = time.time()
        rclpy.spin_once(Camera_depth)
        
        # Capturing and storing left and right camera images

        if type(Camera_depth.img_1) != type(None) and type(Camera_depth.img_0) != type(None) :

            Left_nice= cv2.remap(Camera_depth.img_1,Left_Stereo_Map_x,Left_Stereo_Map_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
            Right_nice= cv2.remap(Camera_depth.img_0,Right_Stereo_Map_x,Right_Stereo_Map_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

            disp = stereo.compute(Left_nice, Right_nice).astype(np.float32)
            disp = cv2.normalize(disp,0,255,cv2.NORM_MINMAX)
            disp = disp[:,100:]

            print(f"disparity at (0,0): {disp[0][0]}")

            print(np.amin(disp))
            print(np.amax(disp))
            
            depth_map = M / disp


            print('-------------------------------------')

            mask_temp = cv2.inRange(depth_map,min_depth,max_depth)
            depth_map = cv2.bitwise_and(depth_map,depth_map, mask= mask_temp)

            print(depth_map)
            print(np.amin(depth_map))
            print(np.amax(depth_map))

            # print(np.amin(depth_map))
            # print(depth_map)

            obstacle_avoid(depth_map, depth_tresh, Camera_depth.out)
            

            cv2.imshow('SGBM', disp)
            print(f"mean : {np.mean(depth_map)}")

            if  cv2.waitKey(1) == ord('q'):
                break

        print(time.time()-start)
        
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Camera_depth.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
