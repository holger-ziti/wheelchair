# force_joystick
 - code to read the 3D force joystick to ROS2 topics
 - code to identify (position and force) joystick user dynamics

### nodes
 - ./force_joystick/src/analog_in_pub.cpp  
   read the Analog Input to BeagleBoneBlack and publish the raw voltage
 - ./force_joystick/scripts/force_cmd.py
   from the raw voltage values (measured force) create a /cmd_vel topi

 


