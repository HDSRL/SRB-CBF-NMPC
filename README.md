# Safety-Critical and Distributed Nonlinear Predictive Controllers for Teams of Quadrupedal Robots
This repository features the code for the paper [B. M. Imran, J. Kim, T. Chunawala, A. Leonessa and K. A. Hamed, "Safety-Critical and Distributed Nonlinear Predictive Controllers for Teams of Quadrupedal Robots," in IEEE Robotics and Automation Letters, vol. 10, no. 9, pp. 9176-9183, Sept. 2025, doi: 10.1109/LRA.2025.3592073](https://ieeexplore.ieee.org/document/11091376)

# Overview
- The paper cited above presents a novel distributed and nonlinear model predictive control using nonlinear SRB template models and distributed real-time safety-enforcing CBF-NMPC controllers for multi-agent quadrupedal robots. (Lab Website: https://www.kavehakbarihamed.com/)
- The code uses RaiSim as simulation platform
- Casadi framework with Ipopt is used for the nonlinear solver

## Code Organization
The bulk of the code and computations is run through the class "LocoWrapper.cpp". However, the main function for initiating RaiSim and the simulations is in src/robots/A1_Sim.cpp. Note that other files in the robots folder may be outdated and are largely used for troublshooting.

## Where to git clone
Clone this repository under raisim_workspace

## Prerequisite
- In bash, the following must be defined
```sh
export WORKSPACE=${HOME}/RAISIM_WORKSPACE
export LOCAL_INSTALL=${HOME}/RAISIM_WORKSPACE/build
```
- Be sure to add your license to the /rsc folder, and change the license name in src/robots/A1_Sim.cpp


## Building and Running Code
### Build Code From Scratch
```sh
cd $WORKSPACE
cd A1_Robot && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$LOCAL_INSTALL
make -j8
```

### Build and Run Code Using Script
```sh
cd $WORKSPACE/SRB_NMPC
./run_Raisim.sh
```

### Run Example from Command Line
```sh
cd $WORKSPACE
cd A1_Robot/build
./A1_Sim 
```

Finally, there are several different checkerboard colors provided in the material file (rsc/material/myMaterials.material) and can be changed in the A1_Sim.cpp file. To generate a different cherckerboard color pattern, use the MATLAB script (rsc/material/ColoredChecker.m) and create a new material in myMaterials.material following the same convention as the other checkerboard materials.

# Citation
This work has been featured as  in IEEE Robotics and Automation Letters as of July 2025: 
[B. M. Imran, J. Kim, T. Chunawala, A. Leonessa and K. A. Hamed, "Safety-Critical and Distributed Nonlinear Predictive Controllers for Teams of Quadrupedal Robots," in IEEE Robotics and Automation Letters, vol. 10, no. 9, pp. 9176-9183, Sept. 2025, doi: 10.1109/LRA.2025.3592073](https://ieeexplore.ieee.org/document/11091376)


If you benefit from the code or work, please remember to cite us:
```
@ARTICLE{11091376,
  author={Imran, Basit Muhammad and Kim, Jeeseop and Chunawala, Taizoon and Leonessa, Alexander and Hamed, Kaveh Akbari},
  journal={IEEE Robotics and Automation Letters}, 
  title={Safety-Critical and Distributed Nonlinear Predictive Controllers for Teams of Quadrupedal Robots}, 
  year={2025},
  volume={10},
  number={9},
  pages={9176-9183},
  keywords={Safety;Quadrupedal robots;Collision avoidance;Legged locomotion;Planning;Real-time systems;Navigation;Lips;Silicon;Computational modeling;Legged robots;motion control;multi-contact whole-body motion planning and control},
  doi={10.1109/LRA.2025.3592073}}

```

# YouTube video
The YouTube link for the experiments and simulations is [here](https://www.youtube.com/watch?v=N0z3zvkmvW4).
