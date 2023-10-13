# ROS Polygon Publisher
A ROS Node made in Python that publishes the coordinates of a polygon drawn on a Window
## Requirements
Language Used: Python3
Modules/Packages used:
* pygame
* math
* rospy
* geometry_msgs
<!-- -->
Install the dependencies:
```bash
pip install -r requirements.txt
```
## Instructions
* Left Click to Add a Point
* Right Click to Delete the Nearest Point
* Press Backspace on Keyboard to delete the Lastest Added Point
* Press 'C' on Keyboard to delete all the points
<!-- -->
Points Published on /polygon with message type as geometry_msgs/Polygon