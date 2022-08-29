import gi

gi.require_version("Gst", "1.0")

from gi.repository import Gst, GLib
from time import sleep

def main():
    '''
    This scripts can be used to easily display the camera's point of view only by using a GStreamer pipeline.
    It doesn't use any part of ROS2 but it is easy to use and doesn't require the use of multiple Node to work.
    It should only be used on a Test as it not really useful in collaboration with other scripts of the package.
    '''

    Gst.init()

    pipeline0 = Gst.parse_launch('nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv ! xvimagesink sync=false')
    pipeline1 = Gst.parse_launch('nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)NV12, framerate=(fraction)60/1 ! nvvidconv ! xvimagesink sync=false')

    pipeline0.set_state(Gst.State.PLAYING)
    pipeline1.set_state(Gst.State.PLAYING)

    # The next statement will display the pipeline on the screen
    # If the user press Ctrl+C, the pipelines will be stopped and the program will exit

    try:
        while True:
            sleep(0.1)
    except KeyboardInterrupt:
        pass

    # Cleaning Pipelines
    pipeline0.set_state(Gst.State.NULL)
    pipeline1.set_state(Gst.State.NULL)

if __name__ == '__main__':
    main()