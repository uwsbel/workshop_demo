# workshop_demo
The Chrono demos are containerized with Docker. Here are some instructions how to set up container on your machine so that you can successfully run the demo.

## Installation
Open a terminal in your machine then run 


## Run the demo
./demo_FSI_SingleWheelTest_VV_mode 17.5 0.3 3
    17.5 is the mass of the wheel
    0.3 is slip ratio that would like to enforce
    3 is just an ID for this slip

./demo_FSI_SingleWheelTest_RealSlope_mode 17.5 15
    17.5 is the mass of the wheel
    15 is the slope angle of the terrain

./demo_ROBOT_Viper_RealSlope 73.0 15
    73.0 is the mass of the rover
    15 is the slope angle of the terrain
