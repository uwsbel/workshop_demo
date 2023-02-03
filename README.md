# workshop_demo
The Chrono demos are containerized with Docker. Here are some instructions how to set up container on your machine so that you can successfully run the demo.

## Installation
Open a terminal in your machine then run 

```git clone https://github.com/uwsbel/workshop_demo.git && cd workshop_demo```

Once getting into folder, running the following to build docker image from Dockerfile:

``` docker build -t <img_name> . ```

Notice that you can put a tag name for the image you build by using flag -t. After building it, run a container using the command:

```docker run -it --gpus all <img_name> ```

You should be get into container. To run the demo, go to directory ``` /root/sbel/chrono/build/bin/```, the executable demos are in this folder.


## Run the demo
- ./demo_FSI_SingleWheelTest_VV_mode 17.5 0.3 3
    - 17.5 is the mass of the wheel
    - 0.3 is slip ratio that would like to enforce
    - 3 is just an ID for this slip

- ./demo_FSI_SingleWheelTest_RealSlope_mode 17.5 15
    - 17.5 is the mass of the wheel
    - 15 is the slope angle of the terrain

- ./demo_ROBOT_Viper_RealSlope 73.0 15
    - 73.0 is the mass of the rover
    - 15 is the slope angle of the terrain
