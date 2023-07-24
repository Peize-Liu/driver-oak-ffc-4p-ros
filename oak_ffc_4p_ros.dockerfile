
FROM luxonis/depthai-ros:noetic-latest
ARG OKA_WS=/root/oak_ffc_ws/
ARG ROS_VERSION=noetic

SHELL ["/bin/bash", "-c"] 
WORKDIR ${OKA_WS}
RUN apt-get update &&\
    apt-get install ros-noetic-depthai-ros-msgs ros-noetic-depthai-bridge
RUN mkdir -p ${OKA_WS}/src &&\
    source "/opt/ros/$ROS_VERSION/setup.bash" &&\
    cd ${OKA_WS}/src &&\
    git clone https://github.com/Peize-Liu/oak-ffc-4p-ros.git &&\
    cd oak-ffc-4p-ros/depthai-core-v2.21.2 &&\
    cmake -S. -Bbuild &&\
    cmake --build build
    # cd ${OKA_WS} &&\
    # catkin_init_workspace &&\
    # catkin_make
WORKDIR ${OKA_WS}
RUN source "/opt/ros/$ROS_VERSION/setup.bash" &&\
    catkin_make

# COPY ./oak_ffc_4p_ros/ ${OKA_WS}/src/oak-ffc-4p-ros/
# WORKDIR ${OKA_WS}
# RUN source "/opt/ros/${ROS_VERSION}/setup.bash" &*&