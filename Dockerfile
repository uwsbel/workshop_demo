FROM nvidia/cuda:11.8.0-devel-ubuntu20.04

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV TZ=US/Central
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ /etc/timezone

RUN DEBIAN_FRONTEND=noninteractive

WORKDIR /root/sbel

#chrono dependency installed here
RUN apt update && apt install -y --no-install-recommends --allow-unauthenticated libeigen3-dev cmake cmake-curses-gui mesa-common-dev wget git ninja-build vim

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
 -DBUILD_DEMOS=OFF \
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
 -DCUDA_ARCH_NAME=All \
 && ninja -j 8 && ninja install

RUN mkdir demos/single_wheel_vv_mode/build
RUN cd demos/single_wheel_vv_mode/build && cmake ../ . -G Ninja \
-DCMAKE_BUILD_TYPE=Release \
-DChrono_DIR=/root/sbel/chrono/build/cmake \
&& ninja

RUN mkdir demos/single_wheel_real_slope_mode/build
RUN cd demos/single_wheel_real_slope_mode/build && cmake ../ . -G Ninja \
-DCMAKE_BUILD_TYPE=Release \
-DChrono_DIR=/root/sbel/chrono/build/cmake \
&& ninja

RUN mkdir demos/viper_real_slope/build
RUN cd demos/viper_real_slope/build && cmake ../ . -G Ninja \
-DCMAKE_BUILD_TYPE=Release \
-DChrono_DIR=/root/sbel/chrono/build/cmake \
&& ninja

RUN mkdir outputs


ENTRYPOINT ["/bin/bash"]
