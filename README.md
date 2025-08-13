# Convex Cover of Collision-free Space for Trajectory Generation

## About

Code implementation for collision-free space approximation as polytopes to cover the same homotopic path or trajectory.

__Authors__: [Yuwei Wu](https://github.com/yuwei-wu), Igor Spasojevic,  Pratik Chaudhari, and Vijay Kumar from the [Kumar Lab](https://www.kumarrobotics.org/).

__Video Links__: [Youtube](https://youtu.be/k7CI7-fgSXE)


__Related Paper__: Y. Wu, I. Spasojevic, P. Chaudhari and V. Kumar, "Towards Optimizing a Convex Cover of Collision-Free Space for Trajectory Generation," in IEEE Robotics and Automation Letters, vol. 10, no. 5, pp. 4762-4769, May 2025


If this repo helps your research, please cite our paper at:

```
@ARTICLE{10935632,
  author={Wu, Yuwei and Spasojevic, Igor and Chaudhari, Pratik and Kumar, Vijay},
  journal={IEEE Robotics and Automation Letters}, 
  title={Towards Optimizing a Convex Cover of Collision-Free Space for Trajectory Generation}, 
  year={2025},
  volume={10},
  number={5},
  pages={4762-4769}}
```

## Acknowledgements


- Evaluation environments: [kr_param_map](https://github.com/KumarRobotics/kr_param_map)
- Front-end Path Planning: We use [OMPL](https://ompl.kavrakilab.org/) planning library
- Planning Modules and Visualization: We use the module in [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER)


## Run in Docker

### 1. Clone the repository

```bash
git clone https://github.com/KumarRobotics/kr_opt_sfc.git
```

### 2. Build the Docker image

Replace `$(whoami)` with your local username if needed.

```bash
docker build --build-arg user_name=$(whoami) -t opt_sfc .
```

### 3. Run the Docker container

Mount your local repo inside the container to `/home/<username>/opt_sfc_ws/src/kr_opt_sfc` for development.

```bash
docker run -it \
    -v $(pwd)/kr_opt_sfc:/home/$(whoami)/opt_sfc_ws/src/kr_opt_sfc \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    opt_sfc
```

### 4. Inside the container

Build and launch your ROS workspace:

```bash
catkin build
source devel/setup.bash
roslaunch opt_sfc sfc.launch
```aunch opt_sfc sfc.launch
```

## Run from Source

The repo has been tested on 20.04 with ros-desktop-full installation.


### 1. Prerequisites

#### 1.1 ROS and OMPL

Follow the guidance to install [ROS](https://wiki.ros.org/ROS/Installation) and install OMPL:
```
sudo apt install libompl-dev
```

### 2. Build on ROS 

```
mkdir -p opt_ws/src
cd opt_ws/src
git clone git@github.com:KumarRobotics/kr_opt_sfc.git
wstool init && wstool merge kr_opt_sfc/utils.rosinstall && wstool update
cd ..
catkin build
```

### 3. Run

```
source devel/setup.bash
roslaunch opt_sfc sfc.launch
```

In "kr_opt_sfc/src/opt_sfc/config/sfc.yaml", change paramters to test different performance. The default is normal mode, if you want to show each iteration, set "sfc_cover_opt:debug=true".

<p align="center">
  <img src="docs/3d1.png" width = "300" height = "170"/>
  <img src="docs/3d2.png" width = "300" height = "170"/>
</p>



### 4. Run in 2D Image Map 


To generate a 2D corridor, you can launch the 2D projection with


```
source devel/setup.bash
roslaunch opt_sfc sfc2d.launch
```

You can use the format specified in the kr_param_map to create the map from image inputs.

In opt_sfc/scripts, there's an example saved in "projected_2d.txt", you can run

```
python visualizer2d.py 

```

to visualize it.

<p align="center">
  <img src="docs/2d1.png" width = "300" height = "170"/>
  <img src="docs/2d2.png" width = "300" height = "170"/>
</p>


## Dataset Generation


You can save the corridor and initial route data for further planning or evaluation.

### 1. Install 


```
sudo apt-get install libhdf5-dev
pip install h5py
```

### 2. Set your path and set "save_sfc" as true

```
  <arg name="sfc_dataset_path" default="$(find opt_sfc)/dataset/"/>
  <arg name="save_sfc" default='true'/>
```

### 3. Run

```
source devel/setup.bash
roslaunch opt_sfc sfc.launch
```

If you want to run different random map, set "auto_change" as true:

```
    <param name="map/auto_change"    value="true"/>
```

### 4. Visualize

In opt_sfc/scripts, you can set the dataset path and run

```
python read_dataset.py 
```

to visualize the polytopes.

### 5. Dataset Structure (.h5)


Each trial is stored under:

```
/trial_000001/   
├── route          # Nx3 float64 matrix of 3D waypoints
└── polys/
├── poly_0000 # Mx4 float64 matrix (half-spaces: ax + by + cz + d ≤ 0)
├── poly_0001 
├── ...
...
/trial_<index>/
```


## Maintaince

For any technical issues, please contact Yuwei Wu (yuweiwu@seas.upenn.edu, yuweiwu20001@outlook.com).
