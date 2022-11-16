# beginner_tutorials

Programming Assignment: ROS Publisher/Subscriber
---

## Dependencies

Ros2 Humble

## Install via command line
check dependencies
```
rosdep install -i --from-path src --rosdistro humble -y
```
Navigate to Workspace SRC folder
```
cd ~/ros2_ws/src
```
Clone Package
```
git clone https://github.com/xtan99/beginner_tutorials.git
```
Build Package in Workspace
```
cd ~/ros2_ws/
colcon build --packages-select beginner_tutorials
```
Source ros2 underlay and workspace overlay

Run Publisher and subscriber in seperate terminals
```
ros2run beginner_tutorials talker
ros2run beginner_tutorials talker
```
Run launch file for publisher and service call in seperate terminals(Week-10)
```
ros2 launch beginner_tutorials talker_launch.yaml my_parameter:=<"String parameter you wish to set for the publisher node">
ros2 service call /print_new_string beginner_tutorials/srv/PrintNewString <"{s: New string to be published}">
```



