import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray

import numpy as np
import cupy as cp
import gi
import time
import gc

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")

from gi.repository import Gst,GstApp, GLib

shape = [360,640,3]
resolution = shape[0]*shape[1]*shape[2]
dev1 = cp.cuda.Device(0)

def NewSample(sample):
    caps = sample.get_caps()

    # Extract the width and height info from the sample's caps
    height = caps.get_structure(0).get_value("height")
    width = caps.get_structure(0).get_value("width")


    # Get the actual data
    buffer = sample.get_buffer()

    # Get read access to the buffer data
    success, map_info = buffer.map(Gst.MapFlags.READ)
    if not success:
        raise RuntimeError("Could not map buffer data!")

    numpy_frame = np.ndarray(shape=(height, width, 3), dtype=np.uint8, buffer=map_info.data)

    # Clean up the buffer mapping
    buffer.unmap(map_info)

    return(numpy_frame)

class CameraPublisher(Node):

    def __init__(self, sink1):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(UInt8MultiArray, 'camera1_data', 10)
        timer_period = 0.065  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sink1 = sink1


    def timer_callback(self):
        msg = UInt8MultiArray()
        msg.data = []

        sample1 = self.sink1.try_pull_sample(Gst.SECOND)
        if sample1 is None:
            self.get_logger().info('sink1: I have no Sample')
        else:
            cu1 = NewSample(sample1)
            with dev1:
                cup1 = cp.reshape(cu1,(resolution))
            data = cup1.tolist()
            assert(type(data) == list and len(data) == resolution)
            msg.data = data

        self.publisher_.publish(msg)
        self.get_logger().info('sink1: I have a Sample: "%s" ... "%s" ... "%s"' % (msg.data[0:6], msg.data[resolution//2],msg.data[resolution-3:resolution-1]))


def main(args=None):
    rclpy.init(args=args)

    Gst.init()

    pipeline1 = Gst.parse_launch("nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360,format=(string)NV12 ! nvvidconv ! videoconvert ! video/x-raw, format = RGB ! appsink name=sink1 max-buffers=1 drop=True")
    
    appsink1 = pipeline1.get_by_name('sink1')

    pipeline1.set_state(Gst.State.PLAYING)

    camera_publisher = CameraPublisher(appsink1)

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pipeline1.set_state(Gst.State.NULL)
    
    # Cleaning Pipelines
    pipeline1.set_state(Gst.State.NULL)
    time.sleep(0.1)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
