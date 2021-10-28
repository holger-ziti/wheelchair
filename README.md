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

### build code
```console
cd ~/ws
colcon build
```

### run code
```console
ros2 launch force_joystick two_analog_in.launch.py
```


