# Installation



Clone this docker and its definition (both in the marine-aerial branch):
```
mkdir ~/Dockers && cd "$_"
git clone --branch marine-aerial git@bitbucket.org:autonomouscv/marine_dev.git
git clone --branch marine-aerial git@bitbucket.org:autonomouscv/marine_dev_docker.git
```

Open the docker using VSCode:
```
cd marine_dev
code .
```
Once in VSCode, press `ctrl+shift+p`, and search and execute `Remote-Containers: Rebuild Container`. Go for a coffee, this will take a while.

Finalise the ROS setup:
```
sudo rosdep init
rosdep update
```

Let's proceed with the installation:
```
cd ~/workspace/src
vcs import < /workspaces/marine_dev/docker_workspace/robots/dev.yml 
vcs import < /workspaces/marine_dev/docker_workspace/robots/nukhada.yml
```

Build the catkin workspace:
```
cd ~/workspace
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

Important: the plugin for simulating the buoyancy only works when built in release mode, so after you built the workspace do:
```
catkin build asv_wave_sim --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Source the catkin workspace:
```
source ~/workspace/devel/setup.zsh
```

Now, you should be able to launch the catamaran:
```
roslaunch dave_demo_launch dave_demo.launch
```
and, independently,
```
roslaunch nukhada_description demo_controller.launch
rosservice call /nukhada/go_to_incremental "step:
  x: 10.0
  y: 0.0
  z: 0.0
max_forward_speed: 5.0
interpolator: ''"
```