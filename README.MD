# beginner_tutorials

Programming Assignment: ROS Publisher/Subscriber
---

## Dependencies

Ros2 Humble

## Install via conmmand line
check dependencies
```
rosdep install -i --from-path src --rosdistro humble -y
```
Build Workspace
```
cd ~/ros2_ws
colcon build
```
Source ros2 underlay and workspace overlay

Run Publisher and subscriber in seperate terminals
```
ros2 run beginner_tutorials talker
ros2 run beginner_tutorials listener 
```



