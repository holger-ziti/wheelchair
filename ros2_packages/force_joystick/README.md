# force joystick


 - code to read the 3D force joystick to ROS2 topics
 - code to identify (position and force) joystick user dynamics

### nodes
 - ./force_joystick/src/analog_in_pub.cpp  
   read the Analog Input to BeagleBoneBlack and publish the raw voltage
 - ./force_joystick/scripts/force_cmd.py
   from the raw voltage values (measured force) create a /cmd_vel topi
   



### build code (maybe twice)
```console
cd ~/ws
colcon build --packages-select force_joystick
```

### run code
This is the default code to run on BBB. 
Force joystick orientation: black cable pointing to the user.
Forces to the left: positive value for axes[0] (to the right: negative)
```console
ros2 launch force_joystick two_analog_in_eloquent.launch.py
```


 
### use joystick for sim

```console
ros2 launch force_joystick force_joystick_foxy.launch.py
```


### wheelchair simulation
```console
ros2 launch force_joystick wheelchair_gazebo_ziti_floor.launch.py
```

### wheelchair sim (PC)
 - schwarzes kabel am kraftsensor nach hinten (<https://github.com/5i0770/wheelchair_gazebo.git>)
 
```console
ros2 launch wheelchair_gazebo wheelchair_gazebo.launch.py
```

