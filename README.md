# wheelchair
code related to the wheelchair research platform 

### ssh into BBB
```console
ssh debian@b3-aut11
cd ~/git
```

### relevant code changes
```console
cd ~/git/wheelchair
git pull
```

### source
```console
source ~/scripts/ros2_ssh.sh
```

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

### do nothing for 30 sec
 - offset is calculated
 - cmd_vel needs some time
 
### use joystick for sim

```console
ros2 launch force_joystick force_joystick_foxy.launch.py
```


### wheelchair simulation
```console
ros2 launch force_joystick wheelchair_gazebo_ziti_floor.launch.py
```

### wheelchair sim (PC)
 - schwarzes kabel am kraftsensor nach hinten
```console
git clone https://github.com/5i0770/mobile_robot_gym.git
ln -s ~/git/mobile_robot_gym/ros2_workspace/src/wheelchair_gazebo ~/dev_ws/src
ros2 launch wheelchair_gazebo worl
```


### set topic /cmd_vel to zero
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'


 


