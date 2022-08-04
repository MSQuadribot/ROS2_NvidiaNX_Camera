#### ROS2 Camera Package for the Nvidia NX Xavier

This package's goal is to use the two camera IMX219 provided with the nvidia NX Xavier in a ROS2 environment

### How to build the package :

First, open a new terminal and go to the following directory : /Desktop/dev_ws
There you will be able to build all the packages that are in the src subdirectory

$ colcon build

This process can take some times (on my machine it could take up to 15s to run)
Remember that if the built was already completed, there is no need to run it again
But, be also aware that you have to rebuild the package each time you make a modification

### How to run the package :

For that, you will need to open a new terminal and go to the following directory : /Desktop/dev_ws
If you just built the package, the terminal will need to be different as this can cause issues otherwise

in order to launch a process for this package you will need to source your workspace

$ . install/setup.bash

Once done, you can process with using this wonderful package

$ ros2 run camera {$processname}

### What are the available process ?

There are currently five available process :

publisher is used to launch the publisher, thus sending frames from both cameras.
They will sent frames from the cameras to the subscribers

result is used to launch a subscribers
They will receive frames sent by the publishers and display them with OpenCV.
The display rate is around 28 fps in the best possible case.

posting is used to send images (.jpg) to a remote server.
The sending rate is around one frame per second.

display is used to launch a simple Python code
It will allow the user to see cameras' stream localy with 60f ps.
