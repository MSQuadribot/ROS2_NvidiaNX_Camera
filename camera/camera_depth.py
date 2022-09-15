from copyreg import dispatch_table
import rclpy
from rclpy.node import Node

import numpy as np
import cv2
import time

import jetson_inference
import jetson_utils

from std_msgs.msg import UInt8MultiArray

shape = [540,960,3]

resolution = shape[0]*shape[1]*shape[2]

def remap(x, in_min, in_max, out_min, out_max):
    '''
    Small and simple function used to remap the data from to joystick to a desired format.
    '''

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def obj_id(item):
    center = (int(item.Center[0]), int(item.Center[1]))
    h = int(item.Height)
    w = int(item.Width)
    x = int(center[0])-w//2
    y = int(center[1])-h//2

    return(x,y,w,h,center)

def obj_detect(item,depth_map,depth_thresh,output):
    '''
    This fonction will retrieve the position of detected object.
    Then it will create a binding between the actaul image and the depth map.
    Thus, it should be easy to display the real distance between the object and the camera.
    '''

    obj = False

    if len(item):
        for e in item:
            x,y,w,h,center = obj_id(e)
            xc, yc = center
            #depth = depth_map[yc:yc+int(0.1*h),xc:xc+int(0.1*w)]
            depth = depth_map[y:y+h,x:x+w]

            mask = np.zeros_like(depth)
            mask = remap(depth, np.amin(depth), np.amax(depth), 0, 255).astype(np.uint8)

            depth_mean, _ = cv2.meanStdDev(depth, mask=mask)
            print(depth_mean)

            start = (x,y)
            end = (x+w,y+h)
            color = (255,0,0)
            thick = 2

            if depth_mean <= depth_thresh :

                    # Display warning text
                    cv2.putText(output, "WARNING !", (x+5,y-40), 1, 2, (0,0,255), 2, 2)
                    cv2.putText(output, "Object at", (x+5,y), 1, 2, (100,10,25), 2, 2)
                    cv2.putText(output, "%.2f cm"%depth_mean, (x+5,y+40), 1, 2, (100,10,25), 2, 2)
                    output = cv2.rectangle(output,start,end,color,thick)

                    obj = True

    if obj == False :
        cv2.putText(output, "SAFE!", (100,100),1,3,(0,255,0),2,3)


def find_obj(depth_map, depth_thresh, output, disp):
    '''
    This function will try to find shapes and object on the disparity map.
    Then, it will use the coordinates of the object to find its distance using the depth map.
    Finally, it will display a warning if an object is near the cameras.
    Even though the object detection is now done by AI, this function was kept as a potential backup.
    '''

    obj = False

    contours, _ = cv2.findContours(disp, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(contours, key=cv2.contourArea, reverse=True)
    n = len(cnts)
    i = 0

    while obj == False and i < n:

        x,y,w,h = cv2.boundingRect(cnts[i])
        start = (x,y)
        end = (x+w,h+h)
        color = (255,0,0)
        thick = 2

        if (x+w)*(y+h) > 0.15*disp.shape[0]*disp.shape[1] and (x+w)*(y+h) < 0.80*disp.shape[0]*disp.shape[1] and h > 0.10*disp.shape[0] and w > 0.10*disp.shape[1]:

            # Calculating the average depth of the object closer than the safe distance

            depth = depth_map[y:y+h,x:x+w]
            depth_mean = (np.amax(depth) + np.mean(depth))/2

            if depth_mean <= depth_thresh :

                # Display warning text
                cv2.putText(output, "WARNING !", (x+5,y-40), 1, 2, (0,0,255), 2, 2)
                cv2.putText(output, "Object at", (x+5,y), 1, 2, (100,10,25), 2, 2)
                cv2.putText(output, "%.2f cm"%depth_mean, (x+5,y+40), 1, 2, (100,10,25), 2, 2)
                output = cv2.rectangle(output,start,end,color,thick)

                cv2.putText(disp, "WARNING !", (x+5,y-40), 1, 2, (0,0,255), 2, 2)
                cv2.putText(disp, "Object at", (x+5,y), 1, 2, (100,10,25), 2, 2)
                cv2.putText(disp, "%.2f cm"%depth_mean, (x+5,y+40), 1, 2, (100,10,25), 2, 2)
                disp = cv2.rectangle(disp,start,end,color,thick)

                obj = True

        i = i+1


    if obj == False:
        cv2.putText(output, "SAFE!", (100,100),1,3,(0,255,0),2,3)
        cv2.putText(disp, "SAFE!", (100,100),1,3,(0,255,0),2,3)

    cv2.imshow('output',output)
    cv2.imshow('SGBM', disp)


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

def main(args=None):
    '''
    This function is called when the user launch the executable file.
    Ros2 will directly start the program here, according to the setup file.
    The loop will use the callback to display the image on the screen.
    '''

    rclpy.init(args=args)

    Camera_depth = CameraDepth()

    # Define some global geometric parameter (cm)

    M = 25 *0.315
    min_depth = 30
    max_depth = 500
    depth_thresh = 500.0

    # Reading the mapping values for stereo image rectification

    cv_file = cv2.FileStorage("/home/qb/Desktop/dev_ws/src/camera/camera/calibration.xml", cv2.FILE_STORAGE_READ)
    Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
    Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
    Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
    Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
    cv_file.release()

    # Setting parameters for StereoSGBM algorithm

    minDisparity = 0
    numDisparities = 96
    blockSize = 7
    disp12MaxDiff= 1
    uniquenessRatio = 5
    speckleWindowSize = 1
    speckleRange = 20

    sigma = 3
    lmbda = 10000.0

    #Creating an object of StereoBM algorithm

    stereo = cv2.StereoBM_create(numDisparities=numDisparities, blockSize=blockSize)
    stereo.setMinDisparity(minDisparity)
    stereo.setDisp12MaxDiff(disp12MaxDiff)
    stereo.setUniquenessRatio(uniquenessRatio)
    stereo.setSpeckleWindowSize(speckleWindowSize)
    stereo.setSpeckleRange(speckleRange)

    # Create a filter that will be applied to the disparity map

    stereoR = cv2.ximgproc.createRightMatcher(stereo)
    wls = cv2.ximgproc.createDisparityWLSFilter(stereo)
    wls.setLambda(lmbda)
    wls.setSigmaColor(sigma)

    # Create a neural network to spot object on the actual image

    net = jetson_inference.detectNet("ssd-mobilenet-v2", threshold=0.35)

    while True :
        
        start = time.time()
        rclpy.spin_once(Camera_depth)

        print('-------------------------------------')
        
        # Capturing and left and right camera images from ROS2 topics

        if type(Camera_depth.img_1) != type(None) and type(Camera_depth.img_0) != type(None) :

            # Apply camera calibration result and turn images to gray scales

            Left_nice_c= cv2.remap(Camera_depth.img_1,Left_Stereo_Map_x,Left_Stereo_Map_y, cv2.INTER_LINEAR)
            Right_nice_c= cv2.remap(Camera_depth.img_0,Right_Stereo_Map_x,Right_Stereo_Map_y, cv2.INTER_LINEAR)
            Left_nice = cv2.cvtColor(Left_nice_c,cv2.COLOR_BGR2GRAY)
            Right_nice = cv2.cvtColor(Right_nice_c,cv2.COLOR_BGR2GRAY)

            # Create Stereo images and divide the result by 2048 to get a proper disparity map

            disp = stereo.compute(Left_nice,Right_nice)
            dispR = stereoR.compute(Left_nice,Right_nice)
            disp = disp/2048
            dispR = dispR/2048

            disp = wls.filter(disp, Left_nice, disparity_map_right=dispR)

            # Resize the images to prevent non-positive value inside the disparity

            disp = disp[5:535,100:955]
            Left_nice = Left_nice[5:535,100:955]
            Left_nice_c = Left_nice_c[5:535,100:955]

            # Remap the actual disparity into an 8-bit image

            amin = np.amin(disp)
            amax = np.amax(disp)
            disp = remap(disp,amin,amax,0,255).astype(np.uint8)
            disp_color = cv2.applyColorMap(disp, cv2.COLORMAP_JET)

            # Create the depth map, taking into account the preceding remaping

            depth_map = M / (disp/255)
            mask_temp = cv2.inRange(depth_map,0,max_depth)
            depth_map = cv2.bitwise_and(depth_map,depth_map, mask= mask_temp)

            # Perform actual object detection using simple AI from Nvidia Jetson package
            # Some manipulation have to be done to create a binding between cuda and opencv/numpy

            img = jetson_utils.cudaFromNumpy(Left_nice_c, isBGR=True)
            img_rgb = jetson_utils.cudaAllocMapped(width=img.width, height=img.height, format='rgb8')
            jetson_utils.cudaConvertColor(img, img_rgb)
            output = net.Detect(img_rgb)

            img = jetson_utils.cudaToNumpy(img_rgb)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            obj_detect(output, depth_map, depth_thresh, img)

            # Display the results

            cv2.imshow('Stereo Vision',img)
            cv2.imshow('disparity', disp)
            
            if  cv2.waitKey(1) == ord('q'):
                break

        print(f'time : {time.time()-start}')
        
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Camera_depth.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
