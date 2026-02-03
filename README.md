# teloperated_ultrasound_demo

### Internet setup
- Every device has subnet `255.255.255.0 (/24)`  
- The leader pc must have ip: `192.168.100.50`  
- The follower pc must have ip: `172.31.1.148` on ethernet port 1 and `192.168.100.49` on the ethernet port 2  
- The KUKA robot has ip `172.31.1.147`  
- The haptic interface has ip: `192.168.100.53`  

### Simple Teleoperation
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


## Run leader
```
source entrypoint.sh
ros2 launch haptic_control haptic_control.launch.py use_fixtures:=true
```

## Run follower
Access to zotac02 pc
```
ssh zotac02@192.168.100.49
```