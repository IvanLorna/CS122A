# MINISUB

The Seadragon minisub built for the University of California, Riverside's student organization: RoboSub UCR. 
The minisub is the proof of concept of the submarine's embedded systems, using a Teensy3.2(arduino), NVidia Jetson TX2(CPU), and XBox controller(Joystick) to emulate the real submarine.
The minisub utilizes Robot Operating System (ROS), Python, Arduino, and C/C++ for our controls.

### Prerequisites

* Ubuntu 18.04
* ROS Melodic
* Arduino IDE
* Teensyduino

## Getting Started

1. On your Ubuntu 18.04 environment, navigate to the official arduino website and install the Arduino IDE 
2. open a terminal and install ROS melodic for desktop following the [official ROS melodic installation guide](http://wiki.ros.org/melodic/Installation/Ubuntu)
3. install ROS arduino packages following the [official Rosserial arduino installation guide](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)

### Initializing ROS and nodes

1. Initialize ROS master node
```
roscore
```

2. Find which device (js#) is the Joystick:
```
ls /dev/input/
```

Note: replace '#' with the number you find

3. Initialize Joystick node
```
ls -l /dev/input/js#
sudo chmod a+rw /dev/input/js#
rosparam set joy_node/dev "/dev/input/js#"
rosrun joy joy_node
```
Note: I initialized an alias in ~/.bashrc to run all this from simply ("SD_JOYSTICK")

4. Verify Joystick Topic Contents
```
rostopic echo /Joy
```
5. Initialize Teensy nodes (see below)

6. Verify Teensy Topics Contents
```
rostopic pub ThrusterStates std_msgs/UInt64 # --once
```
Note: Replace '#' with any 64-bit number you'd like to test with, and the teensy should update the 'thrusters' in a pattern corresponding to the input value

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
