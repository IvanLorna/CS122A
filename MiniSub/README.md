# MINISUB

The Seadragon minisub built for the University of California, Riverside's student organization: RoboSub UCR. 
The minisub is the proof of concept of the submarine's embedded systems, using a Teensy3.2(arduino), NVidia Jetson TX2(CPU), and XBox controller(Joystick) to emulate the real submarine.
The minisub utilizes Robot Operating System (ROS), Python, Arduino, and C/C++ for our controls.

## Getting Started

### Prerequisites

* Ubuntu 18.04
* ROS Melodic
* Arduino IDE
* Teensyduino

### Initializing ROS and nodes

1. Initialize ROS master node
```
roscore
```

2. Find which device (jsX) is the Joystick:
```
ls /dev/input/
sudo jstest /dev/input/jsX
```

6. Initialize Joystick node
```
ls -l /dev/input/jsX
sudo chmod a+rw /dev/input/jsX
rosparam set joy_node/dev "/dev/input/jsX"
rosrun joy joy_node
```
Note: I initialized an alias in ~/.bashrc to run all this from simply ("SD_JOYSTICK")

7. Verify Joystick Topic Contents
```
rostopic echo /Joy
```
8. Initialize Teensy nodes (see below)

9. Verify Teensy Topics Contents
```
rostopic echo /chatter
```

### Initializing Teensy with ROS

1. Open a terminal on any directory and type
```roscore```
2. Go to the Arduino IDE
3. upload code utilizing ROS Publisher/Subscriber functionality
4. Open a terminal and type 
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
Note: replace ttyACM0 with whatever port the Arduino IDE is using to program the Teensy

Note: You will have a lot of terminals open at once. Each one is dedicated to a specific tast, and closing one might cause certain functionality to cease.
