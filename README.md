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

## Run 

The repo has been tested on 20.04 with ros-desktop-full installation.

### 1. Prerequisites

#### 1.1 ROS and OMPL

Follow the guidance to install [ROS](https://wiki.ros.org/ROS/Installation) and install OMPL:
```
sudo apt install libompl-dev
```

### 2. Build on ROS 

```
git clone git@github.com:KumarRobotics/kr_opt_sfc.git && cd kr_opt_sfc/src
wstool init && wstool merge utils.rosinstall && wstool update
catkin build
```

### 3. Run

```
source devel/setup.bash
roslaunch opt_sfc sfc.launch
```

In "kr_opt_sfc/src/opt_sfc/config/sfc.yaml", change paramters to test different performance. 

## Maintaince

For any technical issues, please contact Yuwei Wu (yuweiwu@seas.upenn.edu, yuweiwu20001@outlook.com).
