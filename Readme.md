#### ROS2 Camera Package for the Nvidia NX Xavier

This package's goal is to use the two camera IMX219 provided with the nvidia NX Xavier in a ROS2 environment

### How to build the package :

First, open a new terminal and go to the following directory : {/Desktop/dev_ws}.
There you will be able to build all the packages that are in the src subdirectory.

$ colcon build

This process can take some times (on my machine it could take up to 15s to run).
Remember that if the built was already completed, there is no need to run it again.
But, be also aware that you have to rebuild the package each time you make a modification.

### How to run the package :

For that, you will need to open a new terminal and go to the following directory : {/Desktop/dev_ws}.
If you just built the package, the terminal will need to be different as this can cause issues otherwise.

in order to launch a process for this package you will need to source your workspace.

$ . install/setup.bash

Once done, you can process with using this wonderful package.

$ ros2 run camera {$processname}

### What are the available process ?

There are currently five available process :

publisher is used to launch the publisher, thus sending frames from both cameras.
They will sent frames from the cameras to the various subscribers.
Frames are publish using classic ROS2 UInt8MultiArray.
The frames are provided by gstreamer and are converted to msg by the node.

result is used to launch a subscribers
They will receive frames sent by the publishers and display them with OpenCV.
The display rate is around 28 fps in the best possible case.

posting is used to send images (.jpg) to a remote server.
The sending rate is around one frame per second.

async is also used for the very same purpose.
It is however supposed to use an asynchronous method, even though it does not seem to work properly.

display is used to launch a simple Python code.
It will allow the user to see cameras' stream localy with 60f ps.

streamhost and streamclient are both used to perform video streaming using udp protocol.
Both programs only use simple python scripts with Gstreamer for now. Only works localy.
streamhost will send video flux using udp protocol with adress 127.0.0.1.
Port 5000 is used for camera 0 while port 5001 is used for camera 1.
streamclient then connect to both port using udp and displaying video with gstreamer.

### What are the topics provided by this packages

There is a central topic for this package : camera_data, which is provided by the camera publisher.
This topic is then subscribed by listener, result and posting. It is composed of a bytes array.
This array contains two frames, taken at the same time, from both camera. One is displayed at the top of the other.
The main element of this topic is thus a UintMultiArray that encapsulate both images as a bytes array.

The other nodes are consuming this camera_data to produce simple results. They do not return topics.

Finally, display is not using any ROS2 code, it is just a classic python script implemented inside this package.
This way, it is simpler to use directly and provide a understandable example of what this package provide.

### How can I see the images online? (Only Quadribot)

In order to see the posted images, one must visit the following website : https://rbox.live/image.php.
On this page it is possible to aknowledge what images are currently on the server side.
In order to visualize a specific image in the brower : https://rbox.live/image.php?name={imagename}.
It is important to note that the image name must be followed by a file extension (mostly .jpg)