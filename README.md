# workshop_demo
The Chrono demos are containerized with Docker. Here are some instructions how to set up container on your machine so that you can successfully run the demo.

## Installation
### Method 1 ---- Build from scratch 
Open a terminal in your machine then run 

```git clone https://github.com/uwsbel/workshop_demo.git && cd workshop_demo```

Once getting into folder, running the following to build docker image from Dockerfile:

``` docker build -t <img_name> . ```

Notice that you can put a tag name for the image you build by using flag -t. For example,
``` docker build -t uwsbel/demo . ```


### Method 2 ---- Pulling Our Docker Image

Pulling the Docker image by running:

```docker pull uwsbel/demo```


After building it, run a container using the command:

```docker run -it --gpus all -v <dir_to_store_data>:/root/sbel/outputs uwsbel/demo ```

Note: ```<dir_to_store_data>``` is the directory where you want to store the output data from the demos on your host machine.
Windows user will run something like:```docker run -it --gpus all -v C:\Users\SBEL\demo_output\:/root/sbel/outputs uwsbel/demo```
Linux user will run something like: ```docker run -it --gpus all -v /home/harry/workshop_demo/outputs/:/root/sbel/outputs uwsbel/demo```  
Then, you should be get into container.


## Run the demo
Single wheel test under VV mode

```./demo_FSI_SingleWheelTest_VV_mode 17.5 0.3 3```

- 17.5 is the mass of the wheel
- 0.3 is slip ratio that would like to enforce
- 3 is just an ID for this slip

Sinle wheel test under real slope mode

```./demo_FSI_SingleWheelTest_RealSlope_mode 17.5 15 0.8```

- 17.5 is the mass of the wheel
- 15 is the slope angle of the terrain
- 0.8 is the wheel angular velocity

Full VIPER rover under real slope

```./demo_ROBOT_Viper_RealSlope 73.0 15 0.8```

- 73.0 is the mass of the rover
- 15 is the slope angle of the terrain
- 0.8 is the wheel angular velocity

Curiosity rover on uphill and downhill

```./demo_ROBOT_Curiosity_Uphill 200.0 1.0 1```

- 200.0 is the mass of the rover
- 1.0 is the height of the terrain
- 1 is just an ID for this simulation

## Render the VIPER rover results using Blender
- Download the Blender package here: https://download.blender.org/release/Blender2.91/blender-2.91.0-linux64.tar.xz
- Go to one of the full VIPER rover result folder, e.g /FSI_Viper_RealSlope_SlopeAngle_15
- Copy the script "blender_viper_render.py" and the obj file folder "obj_for_render" to the above result folder
- Render an image using below command (7 is the frame number of the output image)

```/home/weihu/Downloads/blender-2.91.0-linux64/blender --background --python ./blender_viper_render.py 7```

## Render the Curiosity rover results using Blender
- Download the Blender package here: https://download.blender.org/release/Blender2.91/blender-2.91.0-linux64.tar.xz
- Go to one of the Curiosity rover result folder, e.g /FSI_Curiosity_Uphill_1
- Copy the script "blender_curiosity_render.py" and the obj file folder "obj_for_render" to the above result folder
- Render an image using below command (15 is the frame number of the output image)

```/home/weihu/Downloads/blender-2.91.0-linux64/blender --background --python ./blender_curiosity_render.py 15```
