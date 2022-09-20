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
    classID = int(item.ClassID)
    h = int(item.Height)
    w = int(item.Width)
    x = int(center[0])-w//2
    y = int(center[1])-h//2

    return(x,y,w,h,center, classID)

def similar(x,y):
    if x > 0.75 * y and x < 1.25 * y:
        return True
    else :
        return False

def obj_detect(item_l,output_l, item_r, M):
    '''
    This fonction will retrieve the position of detected object.
    Then it will create a binding between the actaul image and the depth map.
    Thus, it should be easy to display the real distance between the object and the camera.
    '''

    obj = False

    list_L = {}
    area_L = {}
    list_R = {}
    area_R = {}
    list_class = []

    if len(item_l):
        for e in item_l:
            x,y,w,h,center, classID = obj_id(e)
            if classID in list_L:
                if area_L[classID] < w*h:
                    list_L[classID] = e
                    area_L[classID] = w*h
            else:
                list_L[classID] = e
                area_L[classID] = w*h
                list_class.append(classID)

    if len(item_r):
        for e in item_r:
            x,y,w,h,center, classID = obj_id(e)
            if classID in list_R:
                if area_R[classID] < w*h:
                    list_R[classID] = e
                    area_R[classID] = w*h
            else:
                list_R[classID] = e
                area_R[classID] = w*h
    
    print(list_L, list_R)

    for c in list_class:
        if c in list_R:
            #if area_L[c] < 1.1*area_R[c] and area_L[c] > area_R[c]:
            xl,yl,wl,hl,centerl, classIDl = obj_id(list_L[c])
            xcl, ycl = centerl
            xr,yr,wr,hr,centerr, classIDr = obj_id(list_R[c])
            xcr, ycr = centerr

            disparity = xcl - xcr
            depth = 10 * M/disparity

            start = (xl,yl)
            end = (xl+wl,yl+hl)
            color = (255,0,0)
            thick = 2

            # Display warning text
            cv2.putText(output_l, "WARNING !", (xl+5,yl-40), 1, 2, (0,0,255), 2, 2)
            cv2.putText(output_l, "Object at", (xl+5,yl), 1, 2, (100,10,25), 2, 2)
            cv2.putText(output_l, "%.2f cm"%depth, (xl+5,yl+40), 1, 2, (100,10,25), 2, 2)
            output_l = cv2.rectangle(output_l,start,end,color,thick)

            print(classIDl, depth)

            obj = True

    # if obj == False :
    #     cv2.putText(output_l, "SAFE!", (100,100),1,3,(0,255,0),2,3)

def obj_detect_bis(item_l,output_l, item_r, M):
    '''
    This fonction will retrieve the position of detected object.
    Then it will create a binding between the actaul image and the depth map.
    Thus, it should be easy to display the real distance between the object and the camera.
    '''

    if len(item_l) and len(item_r):

        for L in item_l:
            xl,yl,wl,hl,centerl, classIDl = obj_id(L)
            xcl = centerl[0]

            disparity = False

            for R in item_r:
                xr,yr,wr,hr,centerr, classIDr = obj_id(R)
                xcr = centerr[0]
                
                if classIDl == classIDr and similar(wl*hl, wr*hr) and (10 * M / (xcl-xcr)) < 500:

                    if disparity == False:
                        disparity = xcl - xcr
                        depth = 10 * M/disparity
                    
                    elif disparity != False and (10 * M / (xcl-xcr)) < depth:
                        disparity = xcl - xcr
                        depth = 10 * M/disparity

                if disparity != False:

                    start = (xl,yl)
                    end = (xl+wl,yl+hl)
                    color = (255,0,0)
                    thick = 2

                    # Display warning text
                    cv2.putText(output_l, "WARNING !", (xl+5,yl-40), 1, 2, (0,0,255), 2, 2)
                    cv2.putText(output_l, "Object at", (xl+5,yl), 1, 2, (100,10,25), 2, 2)
                    cv2.putText(output_l, "%.2f cm"%depth, (xl+5,yl+40), 1, 2, (100,10,25), 2, 2)
                    output_l = cv2.rectangle(output_l,start,end,color,thick)

                    print(classIDl, depth)

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

    while obj == False and i                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                < n:

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

    baseline = 25 # cm
    fov = 160   # °
    f_pix = (960 * 0.5) / np.tan(fov * 0.5 * np.pi/180) # pixel
    M = baseline*f_pix

    # Reading the mapping values for stereo image rectification

    cv_file = cv2.FileStorage("/home/qb/Desktop/dev_ws/src/camera/camera/calibration.xml", cv2.FILE_STORAGE_READ)
    Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
    Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
    Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
    Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
    cv_file.release()

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

            # Perform actual object detection using simple AI from Nvidia Jetson package
            # Some manipulation have to be done to create a binding between cuda and opencv/numpy

            img_l = jetson_utils.cudaFromNumpy(Left_nice_c, isBGR=True)
            img_rgb_l = jetson_utils.cudaAllocMapped(width=img_l.width, height=img_l.height, format='rgb8')
            jetson_utils.cudaConvertColor(img_l, img_rgb_l)
            output_l = net.Detect(img_rgb_l)

            img_l = jetson_utils.cudaToNumpy(img_rgb_l)
            img_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2RGB)

            img_r = jetson_utils.cudaFromNumpy(Right_nice_c, isBGR=True)
            img_rgb_r = jetson_utils.cudaAllocMapped(width=img_r.width, height=img_r.height, format='rgb8')
            jetson_utils.cudaConvertColor(img_r, img_rgb_r)
            output_r = net.Detect(img_rgb_r)

            img_r = jetson_utils.cudaToNumpy(img_rgb_r)
            img_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2RGB)

            obj_detect_bis(output_l, img_l, output_r, M)

            # Display the results

            cv2.imshow('Object Left',img_l)
            cv2.imshow('Object Right',img_r)
            
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