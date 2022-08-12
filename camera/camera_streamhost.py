import gi

gi.require_version("Gst", "1.0")

from gi.repository import Gst, GLib
from time import sleep

# Constant values
width = '1280'      # Standard width allowed by the camera
height = '720'      # Standard height allowed by the camera
format = 'NV12'     # Native format for the computer
framerate = '30/1'  # Higher framerate will cause blurry noise on the video stream
host = '192.168.43.52'  # Note that this is the ip adress of the receiving device ('192.168.43.52' '128.93.18.242')
port0 = '5000'      # Port for the first camera
port1 = '5001'      # Port for the second camera

def main():
    '''
    This program is conceived to stream the video from the two cameras.
    The receiving machine will then launch another program that will received the video.
    The two cameras are connected to different port, otherwise the video will be mixed.
    The stream is created with a Gstreamer pipeline.
    '''
    
    Gst.init()

    pipeline0 = Gst.parse_launch(f'nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width={width}, height={height}, format={format}, framerate={framerate} ! nvvidconv ! nvv4l2h264enc insert-sps-pps=1 maxperf-enable=1 ! h264parse ! rtph264pay pt=96 mtu=1316 config-interval=3 ! udpsink host={host} port={port0} sync=0')
    pipeline1 = Gst.parse_launch(f'nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width={width}, height={height}, format={format}, framerate={framerate} ! nvvidconv ! nvv4l2h264enc insert-sps-pps=1 maxperf-enable=1 ! h264parse ! rtph264pay pt=96 mtu=1316 config-interval=3 ! udpsink host={host} port={port1} sync=0')
    
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