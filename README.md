<a name="readme-top"></a>
# Industrial Robotics
### **Condiment Picker Robot**
![Industrial Robotics Logo](https://user-images.githubusercontent.com/114462972/196307132-19465e11-1b34-4f45-9e88-00a31518239f.png)
Contributors: Li-Ting Tsai, Tun Tun Lin, Justin Sia

## Project Description
The project aims to integrate a plausible application and collaboration of the two robots, DOBOT Magician and xArm5 outside of a factory setting. The implementation decided is to utilise both robots in a dining kitchen application where both robots will be tasked with delivering different choices of condiments based on the user's input. This will be implemented both in simulation using MATLAB and on the real DOBOT Magician robot.

## Installation and Setup Guide
### Simulation
1. Download or git clone the repository into a folder of your choice
2. Install the following add-ons in MATLAB:
    - Simulink
    - Optimization Toolbox
    - Robotics System Toolbox
    - ROS Toolbox
    - Statistics and Machine Learning Toolbox
    - Symbolic Math Toolbox
3. Open MATLAB and add the folder into your MATLAB file path
4. Run 'startup_rvc.m' within the toolbox (Robotics\robot-9.10Small_Modified_20220912_WithVisionTB\robot-9.10Small_Modified\rvctools)
5. Run 'main_simulation.m' which is located in Robotics\Code\Simulation

### Running the DOBOT Magician using Ubuntu 18.04 and ROS Melodic
1. [Install](http://wiki.ros.org/melodic/Installation/Ubuntu) ROS Melodic and setup a catkin workspace using the tutorial [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
2. Download the required DOBOT Magician packages, refer to the guide [here](https://github.com/gapaul/dobot_magician_driver/wiki/Instructions-For-Native-Linux)
3. Git clone the repository onto your linux system
```sh
git clone https://github.com/CharlieeT/Robotics.git
```
* For private repo
```sh
git clone https:yourusername@hithub.com/yourproject.git
```
4. Follow step 2-4 in the simulation section
5. Run 'main_simulation.m' which is located in Robotics\Code\Real Demo

## Troubleshooting
-
## Acknowledgements
-

<p align="right">(<a href="#readme-top">back to top</a>)</p>
