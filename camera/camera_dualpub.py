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

    # Get the actual data
    buffer = sample.get_buffer()

    # Get read access to the buffer data
    success, map_info = buffer.map(Gst.MapFlags.READ)
    if not success:
        raise RuntimeError("Could not map buffer data!")

    _frame = array.array('B')
    _frame.frombytes(map_info.data)

    # Clean up the buffer mapping
    buffer.unmap(map_info)

    return(_frame)

class CameraPublisher(Node):

    def __init__(self, sink0, sink1):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(UInt8MultiArray, 'camera_data', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sink0 = sink0
        self.sink1 = sink1


    def timer_callback(self):

        start = time.time()
        sample0 = self.sink0.try_pull_sample(Gst.SECOND)
        sample1 = self.sink1.try_pull_sample(Gst.SECOND)
        
        if sample0 is None or sample1 is None:
            
            self.get_logger().info('sink: I have no Sample')
            msg = UInt8MultiArray()
            msg.data = []
        
        else:

            cu0 = NewSample(sample0)
            cu1 = NewSample(sample1)
            #cu2 = array.array('H',shape)

            msg = UInt8MultiArray(data = (cu0 + cu1))

        self.publisher_.publish(msg)
        ouga = len(msg.data)
        self.get_logger().info(f'sink: I have a Sample {ouga}: "%s" ... "%s" ... "%s"' % (msg.data[0:3], msg.data[resolution-1:resolution],msg.data[resolution-4:resolution-1]))
        print("Time --- %s seconds ---" % (time.time() - start))


def main(args=None):
    rclpy.init(args=args)

    Gst.init()

    pipeline0 = Gst.parse_launch("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw(memory:NVMM), width=(int)960, height=(int)540,format=(string)NV12 ! nvvidconv ! videoconvert ! video/x-raw, format = RGB ! appsink name=sink0 max-buffers=1 drop=True")
    
    appsink0 = pipeline0.get_by_name('sink0')

    pipeline0.set_state(Gst.State.PLAYING)

    pipeline1 = Gst.parse_launch("nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw(memory:NVMM), width=(int)960, height=(int)540,format=(string)NV12 ! nvvidconv ! videoconvert ! video/x-raw, format = RGB ! appsink name=sink1 max-buffers=1 drop=True")
    
    appsink1 = pipeline1.get_by_name('sink1')

    pipeline1.set_state(Gst.State.PLAYING)

    camera_publisher = CameraPublisher(appsink0, appsink1)

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pipeline0.set_state(Gst.State.NULL)
        pipeline1.set_state(Gst.State.NULL)
    
    # Cleaning Pipelines
    pipeline0.set_state(Gst.State.NULL)
    pipeline1.set_state(Gst.State.NULL)
    time.sleep(0.1)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
