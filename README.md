# wheelchair
code related to the wheelchair research platform 

### ssh into BBB
```console
ssh debian@b3-aut11
cd ~/git
```

### first time: get code
```console
git clone https://github.com/holger-ziti/wheelchair.git
ln -s ~/git/wheelchair/ros2_packages/force_joystick ~/ws/src
```

### update code
```console
cd ~/git/wheelchair
git pull
```

### source
```console
source /home/debian/ros2_eloquent/setup.bash
source /home/debian/ros2_eloquent/local_setup.bash
source /home/debian/ws/install/local_setup.bash
source /home/debian/ws/install/setup.bash
export ROS_DOMAIN_ID=100
```

### build code (maybe twice)
```console
cd ~/ws
colcon build --packages-select force_joystick
```

### run code
```console
ros2 launch force_joystick two_analog_in_eloquent.launch.py
```

### do nothing for 30 sec
 - offset is calculated
 - cmd_vel needs some time
 
### use joystick for sim

```console
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### wheelchair sim (PC)
 - schwarzes kabel am kraftsensor nach hinten
```console
git clone https://github.com/5i0770/mobile_robot_gym.git
ln -s ~/git/mobile_robot_gym/ros2_workspace/src/wheelchair_gazebo ~/dev_ws/src
ros2 launch wheelchair_gazebo worl
```

### reset cmd_vel 
start and stop teleop_keyboard
```console
ros2 run turtlebot3_teleop teleop_keyboard
```



 


