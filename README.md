# HDSRL Code
- Distributed and Nonlinear Model Predictive Control using Reference Models for MultiAgent A1 Robots (Lab Website: https://www.kavehakbarihamed.com/)
- Uses SNOPT as a nonlinear solver for NMPC

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
cd $WORKSPACE/A1_Robot
./run_Raisim.sh
```

### Run Example from Command Line
```sh
cd $WORKSPACE
cd A1_Robot/build
./A1_FastMPC ../params/LL_w_CLF.txt ../params/MPC_params.txt ../params/Walking_params.txt
```
- The inputs into the function call are optional, and may have different names so long as the naming convention is followed (see "Changing System Parameters" below)

## Changing System Parameters
- Open the files in A1_Robot/params
- There are three seperate parameter files. "LL" contains low level parameters, "MPC" contains the MPC parameters, and "Walking" contains the walking parameters.
- Note that new parameter files may created with different names, so long as the following naming convention is adhered to:
    - The low level parameters file must contain "LL" in the name
    - The MPC parameters file must contain "MPC" in the name
    - The walking parameters file must contain "Walking" in the name
- The bottom of the parameters files include information on the parameters that can be used/changed

Here is an example of the contents of a low level parameters file:
```
0.5
700
35
1
1
0.1
1000000
100000000
100
0.8


// =========================================== //
// ======= Ordering and default values ======= //
// =========================================== //
// Default Low Level
mu = 0.7;    // coefficient of friction
kp = 700;   // proportional gain on IO linearization
kd = 40;    // derivative gain on IO linearization
useCLF = 1;  // 1 or 0, indicates whether or not to use the CLF 

// QP Cost
tauPen = 1e0; // input weight
dfPen  = 1e-1; // ||F-F_d|| weight
auxPen = 1e6; // auxiliary var weight
clfPen = 1e8; // clf defect var weight

// QP Constraints
auxMax = 100;  // max aux value
clfEps = 0.8; // CLF convergence tuning variable

```
Notice that the bottom half of the file contains information about the ordering and default values, while the top half is the portion that should actually be changed to alter the behavior. Changing this file does not require that the executable is recompiled. Each text file provided will be read when the executable is called.

## Visualization Parameters
There are a series of parameters at the bottom of the simulation script that allows the user to control the visualization properties in RaiSim, as well as produce a recording automatically, most of which are self explainatory. In particular, the following parameters are available:

```
raisim::gui::showContacts = false;
raisim::gui::showForces = false;
raisim::gui::showCollision = false;
raisim::gui::showBodies = true;

std::string cameraview = "isoside"; // iso, side, top, front, sideiso
bool panX = true;                   // Pan view with robot during walking (X direction)
bool panY = false;                  // Pan view with robot during walking (Y direction)
bool record = false;                // Record?
double startTime = 0*ctrlHz;        // Recording start time
double simlength = 300*ctrlHz;      // Sim end time
double fps = 30;                  
std::string directory = "PATH_TO_OUTPUT_DIRECTORY";
std::string filename = "NAME_OF_VIDEO";
```

Note that the first four parameters, as well as panX and panY, can also be enabled/disabled using the GUI interface during simulation. Another important note is that, when panX or panY are enabled, the user will obtain erratic behavior when clicking on an object to change the view via the mouse. 

Finally, there are several different checkerboard colors provided in the material file (rsc/material/myMaterials.material) and can be changed in the A1_Sim.cpp file. To generate a different cherckerboard color pattern, use the MATLAB script (rsc/material/ColoredChecker.m) and create a new material in myMaterials.material following the same convention as the other checkerboard materials.
# Collision-Aware-DNMPC
# Collision-Aware-DNMPC
