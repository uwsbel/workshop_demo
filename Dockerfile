FROM nvidia/cuda:11.4.0-devel-ubuntu20.04

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV TZ=US/Central
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ /etc/timezone

RUN DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y --no-install-recommends --allow-unauthenticated git curl gnupg2 lsb-release

WORKDIR /root/sbel

#chrono dependency installed here
RUN apt update && apt install -y --no-install-recommends --allow-unauthenticated libnvidia-gl-515 libeigen3-dev cmake cmake-curses-gui libglu1-mesa-dev freeglut3-dev mesa-common-dev wget swig libglfw3 libglfw3-dev x11proto-gl-dev glew-utils git libxxf86vm-dev libglew-dev openmpi-common libopenmpi-dev ninja-build

# Clean up to reduce image size
RUN ldconfig && apt-get autoclean -y && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

RUN git clone --recursive https://github.com/projectchrono/chrono.git -b release/8.0
RUN git clone https://github.com/uwsbel/public-metadata.git
# RUN cp -f public-metadata/2022/CRM_Vs_NASA_Exp/sim_scripts/ChBce.cu chrono/src/chrono_fsi/physics/ChBce.cu &&
#     cp -f public-metadata/2022/CRM_Vs_NASA_Exp/sim_scripts/ChBce.cuh chrono/src/chrono_fsi/physics/ChBce.cuh &&
#     cp -f public-metadata/2022/CRM_Vs_NASA_Exp/sim_scripts/ChSystemFsi.cpp chrono/src/chrono_fsi/ChSystemFsi.cpp &&
#     cp -f public-metadata/2022/CRM_Vs_NASA_Exp/sim_scripts/ChSystemFsi.h chrono/src/chrono_fsi/ChSystemFsi.h &&
#     cp -f public-metadata/2022/CRM_Vs_NASA_Exp/sim_scripts/Viper.cpp chrono/src/chrono_models/robot/viper/Viper.cpp &&
#     cp -f public-metadata/2022/CRM_Vs_NASA_Exp/sim_scripts/Viper.h chrono/src/chrono_models/robot/viper/Viper.h &&


RUN mkdir chrono/build
RUN cd chrono/build && cmake ../ -G Ninja \
 -DCMAKE_BUILD_TYPE=Release \
 -DBUILD_BENCHMARKING=OFF \
 -DBUILD_DEMOS=ON \
 -DBUILD_TESTING=OFF \
 -DENABLE_MODULE_IRRLICHT=OFF \
 -DENABLE_MODULE_POSTPROCESS=OFF \
 -DENABLE_MODULE_PYTHON=OFF \
 -DENABLE_MODULE_SENSOR=OFF \
 -DENABLE_OPENMP=OFF \
 -DUSE_FSI_DOUBLE=OFF \
 -DENABLE_MODULE_VEHICLE=ON \
 -DENABLE_MODULE_FSI=ON \
 -DEigen3_DIR=/usr/lib/cmake/eigen3 \
 && ninja && ninja install

ENTRYPOINT ["/bin/bash"]