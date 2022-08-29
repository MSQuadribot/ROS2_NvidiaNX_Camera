import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray

import gi
import time
import array

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")

from gi.repository import Gst,GstApp, GLib

shape = [540,960,3]
resolution = shape[0]*shape[1]*shape[2]

def NewSample(sample):
    '''
    This function is called everytime the appsink has a new sample.
    It is used to get the image from the appsink to publish it.
    It will return a byte array with the image data.
    '''

    # Get the actual data
    buffer = sample.get_buffer()

    # Get read access to the buffer data
    success, map_info = buffer.map(Gst.MapFlags.READ)
    if not success:
        raise RuntimeError("Could not map buffer data!")

    # Creat an array and fill it with the actual data
    _frame = array.array('B')
    _frame.frombytes(map_info.data)

    # Clean up the buffer mapping
    buffer.unmap(map_info)

    return(_frame)

class CameraPublisher(Node):

    def __init__(self, sink0):
        '''
        This function is called when the node is created.
        The node will publish raw data from the camera.
        The subsequent topic is named "camera_data".
        '''

        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(UInt8MultiArray, 'camera_data', 10)
        timer_period = 0.04  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sink0 = sink0


    def timer_callback(self):
        '''
        This callback is called by the node everytime it has to publish a new image.
        Thus, images are published at a constant rate.
        Sample from both camera are joined together to create a single 1D array.
        '''

        start = time.time()
        sample0 = self.sink0.try_pull_sample(Gst.SECOND)
        #sample1 = self.sink1.try_pull_sample(Gst.SECOND)
        
        if sample0 is None:
            
            self.get_logger().info('sink: I have no Sample')
            msg = UInt8MultiArray()
            msg.data = []
        
        else:

            cu0 = NewSample(sample0)
            #cu1 = NewSample(sample1)

            msg = UInt8MultiArray(data = cu0)

        self.publisher_.publish(msg)
        length = len(msg.data)
        self.get_logger().info(f'sink: I have a Sample of length {length}: "%s" ... "%s" ... "%s"' % (msg.data[0:3], msg.data[resolution-1:resolution],msg.data[resolution-4:resolution-1]))
        print("Time --- %s seconds ---" % (time.time() - start))


def main(args=None):
    '''
    This function is called when the user launch the executable file.
    Ros2 will directly start the program here, according to the setup file.
    '''

    # Inits both rclpy and gstreamer

    rclpy.init(args=args)

    Gst.init()

    # Start the program by creating both Gstreamer Pipelines

    pipeline0 = Gst.parse_launch("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw(memory:NVMM), width=(int)960, height=(int)540,format=(string)NV12 ! nvvidconv ! videoconvert ! video/x-raw, format = RGB ! appsink name=sink0 max-buffers=1 drop=True")
    appsink0 = pipeline0.get_by_name('sink0')
    pipeline0.set_state(Gst.State.PLAYING)

    # Create a new CameraPublisher node

    camera_publisher = CameraPublisher(appsink0)

    # Run the rclpy loop, which will take care of calling the timer_callback method every 0.045 seconds
    # In case the user press Ctrl+C, the rclpy loop will be stopped and the program will exit

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    
    # Cleaning Pipelines
    pipeline0.set_state(Gst.State.NULL)
    time.sleep(0.1)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()