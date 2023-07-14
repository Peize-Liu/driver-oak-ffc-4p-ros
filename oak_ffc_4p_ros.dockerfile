
FROM luxonis/depthai-ros:noetic-latest
ARG OKA_WS=/root/oak_ffc_ws

WORKDIR ${OKA_WS}
RUN git clone 