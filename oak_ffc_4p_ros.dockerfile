
FROM luxonis/depthai-ros:noetic-latest
ARG OAK_WS=/root/oak_ffc_ws/
ARG ROS_VERSION=noetic

SHELL ["/bin/bash", "-c"] 
WORKDIR ${OAK_WS}
RUN apt-get update &&\
    apt-get install ros-noetic-depthai-ros-msgs ros-noetic-depthai-bridge

COPY ./  ${OAK_WS}/src/oak-ffc-4p-ros/

RUN cd /root/oak_ffc_ws/src/oak-ffc-4p-ros/depthai-core-v2.21.2 &&\
    cmake -S. -Bbuild &&\
    cmake --build build &&\
    source "/opt/ros/noetic/setup.bash" &&\
    cd ${OAK_WS}/src &&\
    catkin_init_workspace &&\
    cd ${OAK_WS} &&\
    catkin_make
RUN source "/root/oak_ffc_ws/devel/setup.bash" &&\
    echo "source /root/oak_ffc_ws/devel/setup.bash" >> /root/.bashrc


# RUN mkdir -p ${OKA_WS}/src &&\
#     source "/opt/ros/$ROS_VERSION/setup.bash" &&\
#     cd ${OKA_WS}/src &&\
#     git clone https://github.com/Peize-Liu/oak-ffc-4p-ros.git &&\
#     cd oak-ffc-4p-ros/depthai-core-v2.21.2 &&\
#     cmake -S. -Bbuild &&\
#     cmake --build build
#     # cd ${OKA_WS} &&\
#     # catkin_init_workspace &&\
#     # catkin_make



# RUN source "/opt/ros/$ROS_VERSION/setup.bash" &&\
#     catkin_make

# COPY ./oak_ffc_4p_ros/ ${OKA_WS}/src/oak-ffc-4p-ros/
# WORKDIR ${OKA_WS}
# RUN source "/opt/ros/${ROS_VERSION}/setup.bash" &*&