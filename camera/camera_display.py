from time import sleep

import gi

gi.require_version("Gst", "1.0")

from gi.repository import Gst, GLib

Gst.init()

pipeline0 = Gst.parse_launch('nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! xvimagesink sync=false')
pipeline1 = Gst.parse_launch('nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! xvimagesink sync=false')

pipeline0.set_state(Gst.State.PLAYING)
pipeline1.set_state(Gst.State.PLAYING)

### Le Try/except pose certain soucis lorsque l'on utilise Ctrl-C

try:
   while True:
       sleep(0.1)
except KeyboardInterrupt:
    pipeline0.set_state(Gst.State.NULL)
    pipeline1.set_state(Gst.State.NULL)

# Cleaning Pipelines
pipeline0.set_state(Gst.State.NULL)
pipeline1.set_state(Gst.State.NULL)
