# KHI_robot_ros_client
This client is created with QT Framework to send and receive data with KHI_robot package for ROS
***

Set signals wit `set_signals` - is like Ethernet/IP interface, but number of signals,  that can be set  
with `set_signals` is limited with string length  

The fastest and the most useful service is `AS` - you can use it like telnet_client, its working average  
time is 120ms (`set_variable` and `get_variable` are using this service)  

`Move or stop robot` button sets some signal in robots memory

AS service cannot run in two threads so send and receive data are serial  

The main poin to use services is the fact that you can get responce from robot  

`Write to TXT` - writes received data string to file.txt

Also in this client you can use telnet to communicate with robot  

Please if you want to use it install khi_robot following installation manual:  

## KHI_robot installation manual
1. Install ROS Noetic (2020) http://wiki.ros.org/noetic/Installation/Ubuntu
2. In bashrc write ROS source /home/ilya/catkin_ws/devel/setup.bash
3. Create catkin workspace http://wiki.ros.org/catkin/Tutorials/create_a_workspace
4. In catkinws src git clone https://github.com/Kawasaki-Robotics/khi_robot.git
5. Install moveit! 
6. catkin_make
7. Install qt for ROS add there catkin_ws
8. Install catkin_tools https://catkin-tools.readthedocs.io/en/latest/installing.html
9. Delete catkin_ws/src/build и /devel
10. Build in QT
11. realtime https://github.com/Kawasaki-Robotics/khi_robot/blob/master/docs/ConnectingRealRobot.md
12. Install ROS controllers https://github.com/ros-controls/ros_controllers
13. Delete four_wheels_steering and build
14. Command to run for checkup: roslaunch khi_robot_bringup rs007l_bringup.launch simulation:=true
roslaunch khi_rs007l_moveit_config moveit_planning_execution.launch 
15. https://github.com/ros-controls/ros_control Also install this  
16. Add this package

If you ran into some difficulties you can mail me: davida3334@gmail.com  

## Command Service

Service "khi_robot_command_service" is available.

Service format is below...

```text
string type
string cmd
---
int32 driver_ret
int32 as_ret
string cmd_ret
```

Command format is below...

### Execute AS Language Command

```text
string type -> "as"
string cmd -> AS Language Command
---
int32 driver_ret -> driver's return code. Refer KRNX_E_*** in krnx.h
int32 as_ret -> AS return code. Refer AS manual.
string cmd_ret -> Response of AS Language Command
```

### Get Signal Status of [NUM]

```text
string type-> "driver"
string cmd -> "get_signal [NUM]"
---
int32 driver_ret -> driver's return code. Refer KRNX_E_*** in krnx.h
int32 as_ret -> AS return code. Refer AS manual.
string cmd_ret -> "-1"(ON) or "0"(OFF)
```

[NUM] range (depended on AS system setting)

```text
Output: 1~512
Input: 1001~1512
Internal: 2001~2512
```

### Set Output Signal Status of [NUM]

```text
string type -> "driver"
string cmd -> "set_signal [NUM], ..."
---
int32 driver_ret -> driver's return code. Refer KRNX_E_*** in krnx.h
int32 as_ret -> AS return code. Refer AS manual.
string cmd_ret -> NOT USED
```

[NUM] range (depended on AS system setting)

```text
Output: -1~-512, 1~512
Internal: -2001~-2512, 2001~2512
(Positive value indicates ON state, and negative value does OFF state.)
```
