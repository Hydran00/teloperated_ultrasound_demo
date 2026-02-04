# teloperated_ultrasound_demo

### Internet setup
- Every device has subnet `255.255.255.0 (/24)` and is connected through a switch. 
- The leader pc must have ip: `192.168.100.50`  
- The follower pc must have ip: `172.31.1.148` on ethernet port 1 and `192.168.100.49` on the ethernet port 2  
- The KUKA robot has ip `172.31.1.147`  
- The haptic interface has ip: `192.168.100.53`  

### Teleoperation setup
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
- Calibrate haptic interface 
```
ros2 launch haptic_control auto_calibration.launch.py
```

## Camera reconstruction system
Build `torchure_smplx`
Set path to torch, open3d and CUDA
```
# cuda
CUDA_VERSION=<your_cuda_version>
export PATH="/usr/local/cuda-${CUDA_VERSION}/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda-${CUDA_VERSION}/lib64:$LD_LIBRARY_PATH"

# open3d
export Open3D_DIR=<path_to_your_open3d_build>/lib/cmake/Open3D

# torch 
export Torch_DIR=<path_to_your_libtorch>/share/cmake/Torch
```
Build torchure_smplx
```
cd leader/zed_ws/src/torchure_smplx
mkdir build && cd build
cmake ..
make
```
Build zed workspace
```
export torchure_smplx_DIR=<this-repo-path>/leader/zed_ws/src/torchure_smplx/build/
colcon build --symlink-install --packages-select smpl_ros /
    --cmake-args -DCMAKE_PREFIX_PATH=<path_to_your_libtorch>/share/cmake/Torch /
    -DCMAKE_BUILD_TYPE=Release
colcon build --symlink-install
```
## Ultrasound Streaming
Build `clarius_ws`
```
cd clarius_ws/
colcon build --symlink-install
```
Connect to the US probe with the Clarius App and check the IP address and streaming port assigned to the probe. The port should be `5828` by default. Then set the right values in `leader/clarius_ws/launch/us_stream.launch.py` file.
```
{"us_image_topic_name": "us_image"},
{"frame_id": "clarius_probe"},
{"ip_address": "10.160.50.119"},
{"port": 5828},
```
Enable traffic on the streaming port
```
sudo ufw allow <port> # 5828
```

## Run leader
Source every workspace
```
source entrypoint.sh
```
Run haptic interface
```
ros2 launch haptic_control haptic_control.launch.py use_fixtures:=true
```
Then launch the reconstruction system
```source entrypoint.sh
ros2 run smpl_ros zed_fusion_smpl_tracking --ros-args --params-file src/smpl_ros/config/node_params.yaml
```
Activate the ultrasound streaming
```
ros2 launch clarius_ros2 us_stream.launch.py
```
ssh zotac02@192.168.100.49
```
Launch controller
```
ros2 launch kuka_control ultrasound.launch.py ctrl:=kuka_clik_controller model:=iiwa14 
```
Then launch the ImpedanceControl from the teach pendant pressing the play button.

