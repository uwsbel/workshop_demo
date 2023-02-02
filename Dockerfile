FROM nvidia/cuda:11.4.0-devel-ubuntu20.04

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV TZ=US/Central
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ /etc/timezone

RUN DEBIAN_FRONTEND=noninteractive

WORKDIR /root/sbel

#chrono dependency installed here
RUN apt update && apt install -y --no-install-recommends --allow-unauthenticated libeigen3-dev cmake cmake-curses-gui mesa-common-dev wget git ninja-build

# Clean up to reduce image size
RUN ldconfig && apt-get autoclean -y && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

RUN git clone --recursive https://github.com/projectchrono/chrono.git -b release/8.0
RUN git clone https://github.com/uwsbel/workshop_demo.git
RUN cp -f workshop_demo/chrono_source_files/Ch* chrono/src/chrono_fsi/ && \
  cp -f workshop_demo/chrono_source_files/Viper.cpp chrono/src/chrono_models/robot/viper/
RUN mv workshop_demo/demos . && rm -rf workshop_demo

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

ENTRYPOINT ["/bin/bash cd demos"]