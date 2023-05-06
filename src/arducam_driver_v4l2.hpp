#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <list>
#include <vector>
#include <linux/videodev2.h>
#include <memory.h>
#include <unistd.h>
#include <time.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

#include "depthai/depthai.hpp"


namespace OAKFFC4PSDK{
class CameraNode{
 public:
  enum CameraType{
    Color,
    Mono
  };

  CameraNode(std::string& camera_stream_name, CameraType camera_type):stream_name(camera_stream_name),camera_type_(camera_type){};
  ~CameraNode(){};
  int32_t Init(dai::Pipeline & pipeline, dai::CameraBoardSocket board_socket,
              dai::MonoCameraProperties::SensorResolution resolution, uint32_t buffer_size, bool block_mode);
  int32_t Init(dai::Pipeline & pipeline, dai::CameraBoardSocket board_socket,
              dai::ColorCameraProperties::SensorResolution resolution,uint32_t buffer_size, bool block_mode);
  int32_t SetFatherDevice(std::shared_ptr<dai::Device> father_device){
    father_device_ptr_ = father_device;
    return 0;
  };
  int32_t GrabImage(cv::Mat& image);

  std::string stream_name;
 private:
  int32_t is_init_ =0;
  CameraType camera_type_;
  dai::CameraBoardSocket board_socket_;
  std::shared_ptr<dai::Device> father_device_ptr_ = nullptr;

  std::shared_ptr<dai::node::XLinkOut> xlink_out_ptr_ = nullptr;
  
  std::shared_ptr<dai::DataOutputQueue> output_queue_ptr_ = nullptr;
 
  std::shared_ptr<dai::node::MonoCamera> mono_cam_ptr_ = nullptr;
  dai::MonoCameraProperties::SensorResolution mono_resolution_;

  std::shared_ptr<dai::node::ColorCamera> color_cam_ptr_ = nullptr;
  dai::ColorCameraProperties::SensorResolution color_resolution_;
};




struct OAKFFC4PConfig {
  bool raw8 = true;
  bool show = false;
  bool publish_splited = false;
  bool print_clearness = false;
  bool is_sync = false;
  bool is_color = true;
  int32_t camera_amount = 4;
  int32_t buffer_frame_num = 2;
  int32_t fps =20;
  int32_t width = 5120;
  int32_t height = 800;
  int32_t cap_device = 0;
  int32_t camera_num = 4;
  int32_t exposure = 300;
  int32_t gain = 1;
  std::vector<std::string> name_vec = {"CAM_A","CAM_B","CAM_C","CAM_D","CAM_E","CAM_F"};
};

class OAKFFC4PDriver {
 public:
  OAKFFC4PDriver();
  ~OAKFFC4PDriver();
  int32_t Init(ros::NodeHandle & nh);
 private:  
  void grabThread();
  void grab();
  void grabRos(const ros::TimerEvent & event);
  void showImage(cv::Mat & show);
  int32_t init_flag_= -1;
  int32_t cam_fd_ = -1;
  int frame_count_ = 0;

  OAKFFC4PConfig config_;
  std::vector<image_transport::Publisher> pub_splited_;


  std::shared_ptr<dai::Device> device_ptr_ = nullptr;
  std::vector<CameraNode> oak_cam_list_;
  std::vector<cv::Mat> cv_image_vec_;

  //ros
  image_transport::ImageTransport* image_transport_ = nullptr;
  std::thread grab_thread_;
  image_transport::Publisher pub_raw_;
  ros::Time tstart_;
  ros::Timer grab_timer_;  
};
}
