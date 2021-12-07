# wheelchair
pratical information using BBB. 

## ROS2 code is auto-started on boot
see
```console
~/scripts/ros2_curtis_run.sh
```

in general, when working via ssh, source via bash script:
```console
source ~/scripts/ros2_ssh.sh
```

After that, you can "see" the active topics etc.  
For example:
```console
ros2 topic list
```


### via ziti lan
ssh into BBB (PW: Debian2020)
```console
ssh debian@b3-aut11
cd ~/git
```



### via router on ottobock
IP?






## Notes / Todos
- set topic /cmd_vel to zero
```console
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```
- show all active IPs (router on ottobock)
```console
nmap -sP 10.0.0.0/24
```

- copy folders via ssh
1) If you want to copy a directory from machine a to b while logged into a:
```console
scp -r /path/to/directory user@ipaddress:/path/to/destination
```
2) If you want to copy a directory from machine a to b while logged into b:
```console
scp -r user@ipaddress:/path/to/directory /path/to/destination
```

 


