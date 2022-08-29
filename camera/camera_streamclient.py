import gi

gi.require_version("Gst", "1.0")

from gi.repository import Gst, GLib
from time import sleep

# Constant values

host = '192.168.9.1'  # Note that this is the ip adress of the receiving device
port0 = '5000'      # Port for the first camera
port1 = '5001'      # Port for the second camera

def main():
    '''
    This program is conceived to retreive the stream sent by the camera_streamhost.py program.
    The Gstreamer pipeline is created to retreived the video sent via UDP protocol.
    This program must be launched on the receiving machine.
    '''
    
    Gst.init()

    pipeline0 = Gst.parse_launch(f'udpsrc uri=udp://{host}:{port0} port={port0} ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! xvimagesink sync=0')
    pipeline1 = Gst.parse_launch(f'udpsrc uri=udp://{host}:{port1} port={port1} ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! xvimagesink sync=0')
    
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
