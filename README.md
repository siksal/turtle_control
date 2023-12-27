# The turtle_control package
This ROS 2 package controls the turtle in the turtle simulator. It contains all the ROS 2 files for the [ROS 2 for Beginners](https://www.youtube.com/playlist?list=PL50Qb16q3h5r9p0SKCIeaUDLFRZs7MtG3) course.

## Install & Build
Create ROS 2 overlay:
```
mkdir -p turtle_ws/src
cd turtle_ws/src
```
Clone repo:
```
git clone https://github.com/siksal/turtle_control.git
cd ..
```
Build:
```
colcon build
. install/setup.bash
```

## Example
Run the turtle catcher:
```
ros2 launch turtle_control turtle_catcher.launch.py
```
![turtle-catcher](https://github.com/siksal/turtle_control/blob/main/media/turtle_catcher.gif)