# OpenArms_Research

We put cameras in the middle of the palm of existing OpenArms.

And made it possible to perform different actions according to the object through real-time object recognition.

We received a gold prize with this project at the Wearable Computer Contest (WCC) hosted by KAIST.
## Requirements
#### Hardware
* Raspberry pi 3  (with **Ubuntu Mate** )
* Arduino nano

#### Software
* Opencv >= 3.2.0 
* Tensorflow >= 1.1.0
* Keras >= 2.0.8
* ROS kinetic
* rosserial_arduino (arduino ros module)

You should **enable gstreamer** when you build **opencv**!!

## Components
| Sources               |  Explanation                                            |
|-----------------------|---------------------------------------------------------|
| ROS_modeule/          | Folder contains tiny yolo model and pretrained weights. |
|   | MobileNet model with keras.                             |

## Installation
Step 1 : Copy openarms folder in ROS_module folder to your catkin workspace.

Step 2 : Upload Arduino code to your Arduino.

That's all !!

## Quick Start
Step 1 : Execute roscore.
```
roscore
```
Step 2 : Launch ros module.
```
roslaunch openarms detection.launch
```
After 1-2 minutes, ready message will out on your screen.

Step 3 : Start ros serial communication!
```
rosrun rosserial_python serial_node.py /port/you/connected
```
Default setting of port/you/connected is maybe /dev/ttyUSB0.
```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```

## Contact
Any questions about our project are welcome!!

Please contact us!

Anthony Kim : artit.anthony@gmail.com

Ethan Kim : 4artit@gmail.com

## GNU General Public License v3.0
Get more information about [license](https://github.com/ARTITLABS/OpenArms_Research/blob/master/LICENSE)