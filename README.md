# teloperated_ultrasound_demo

### Internet setup
- Every device has subnet `255.255.255.0 (/24)`  
- The leader pc must have ip: `192.168.100.50`  
- The follower pc must have ip: `172.31.1.148` on ethernet port 1 and `192.168.100.49` on the ethernet port 2  
- The KUKA robot has ip `172.31.1.147`  
- The haptic interface has ip: `192.168.100.53`  

### Installation
- Clone this repository
- Init submodules
```
git submodule update init
git submodule update update --recursive # this should download also `lbr-stack` module repository under `kuka_control`
```
- Compile haptic interface nodes
```
cd leader/haptic_interface_ROS2
colcon build --symlink-install
```
- Compile kuka workspace (useful to visualize in rviz KUKA meshes)
```
cd leader/kuka_control
colcon build --symlink-install
```

### Run leader
```
source entrypoint.sh
ros2 launch haptic_control haptic_control.launch.py
```