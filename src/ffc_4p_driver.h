#include <stdio.h>
#include <string.h>
#include <vector>
#include <thread>

#include "depthai/depthai.hpp"

namespace OAKCAM{

class FFC4PDriver
{  
 public:
   struct FFCCameraConfig
   {
      dai::CameraBoardSocket socket;
      dai::ColorCameraProperties::SensorResolution resolution = dai::ColorCameraProperties::SensorResolution::THE_720_P;
      std::string stream_name;
      bool is_master;
      FFCCameraConfig(dai::CameraBoardSocket sck,dai::ColorCameraProperties::SensorResolution res, std::string name,bool master):
         socket(sck),resolution(res),stream_name(name),is_master(master){};
   };
   struct ImageNode
   {
      std::shared_ptr<dai::DataOutputQueue> data_output_q = nullptr;
      std::string topic;
      ImageNode( std::shared_ptr<dai::DataOutputQueue> data_output_q,std::string topic):data_output_q(data_output_q),topic(topic){
      }
   };
   
   std::vector<FFCCameraConfig> CameraList = 
      {{dai::CameraBoardSocket::CAM_A,dai::ColorCameraProperties::SensorResolution::THE_720_P, std::string("CAM_A"), false},
      {dai::CameraBoardSocket::CAM_B,dai::ColorCameraProperties::SensorResolution::THE_720_P, std::string("CAM_B"), false},
      {dai::CameraBoardSocket::CAM_C,dai::ColorCameraProperties::SensorResolution::THE_720_P, std::string("CAM_C"), true},
      {dai::CameraBoardSocket::CAM_D,dai::ColorCameraProperties::SensorResolution::THE_720_P, std::string("CAM_D"),false}};
   FFC4PDriver();
   ~FFC4PDriver();
   int32_t InitPipeline();
   int32_t SetAllCameraSychron();
   int32_t SetVedioOutputQueue();
   void GrabImgThread();
 private:
   std::shared_ptr<dai::Pipeline> pipeline_ = nullptr;
   std::shared_ptr<dai::Device> device_ = nullptr;
   std::list<ImageNode> image_queue_;
   int32_t device_is_detected_ = 0;
   int32_t pipeline_is_init_ = 0;
};
}