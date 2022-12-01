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
Run Talker and the following command in seperate terminals to publish and inspect TF(Week-11)
```
ros2 run beginner_tutorials talker mystaticturtle 0 0 1 0 0 0
ros2 topic echo /tf_static
```
Run Ros tests with and without verbose output(Week-11)
```
colcon test --packages-select beginner_tutorials
colcon test --event-handlers console_direct+ --packages-select beginner_tutorials
```

Run the following command to enable bag file recording(Week-11)
```
ros2 launch beginner_tutorials launch_bag.launch.XML b:=True
rs2 bag info bag_output
```
Run the following commands launch listener and play back bag files in different terminals(Week-11)
```
ros2 run beginner_tutorials listener
ros2 bag play bag_output
```