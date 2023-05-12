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

#include "ffc_4p_driver.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <depthai/utility/Clock.hpp>

namespace OAKCAM{
FFC4PDriver::FFC4PDriver(std::shared_ptr<ros::NodeHandle>& nh){
	if(nh ==nullptr){
		ROS_ERROR("Init with a invalid Nodehandler");
		return;
	}
	this->ros_node_ = nh;
  ROS_INFO("FFC 4P Device Detecting\n");
  auto deviceInfoVec = dai::Device::getAllAvailableDevices();
  const auto usbSpeed = dai::UsbSpeed::SUPER_PLUS;
  auto openVinoVersion = dai::OpenVINO::Version::VERSION_2021_4;
	if(deviceInfoVec.size() != 1 ){
		ROS_ERROR("Multiple devices or No device detected\n");
		this->device_is_detected_ = 0;
		return;
	}
	this->device_ = std::make_shared<dai::Device>(openVinoVersion, deviceInfoVec.front(), usbSpeed);
	if(device_ == nullptr){
		ROS_ERROR("device init failed\n");
		return;
	}
	//print device infomation
	std::cout << "===Connected to " << deviceInfoVec.front().getMxId() << std::endl;
	auto mxId = this->device_->getMxId();
	auto cameras = this->device_->getConnectedCameras();
	auto usbSpeed_dev = this->device_->getUsbSpeed();
	auto eepromData = this->device_->readCalibration2().getEepromData();
	std::cout << "   >>> MXID:" << mxId << std::endl;
	std::cout << "   >>> Num of cameras:" << cameras.size() << std::endl;
	std::cout << "   >>> USB speed:" << usbSpeed_dev << std::endl;
	if(eepromData.boardName != "") {
			std::cout << "   >>> Board name:" << eepromData.boardName << std::endl;
	}
	if(eepromData.productName != "") {
			std::cout << "   >>> Product name:" << eepromData.productName << std::endl;
	}
	this->device_is_detected_ = 1;
	ROS_INFO("FFC 4P Device detected!\n");
	this->GetParameters(*nh);
}

FFC4PDriver::~FFC4PDriver(){
	this->device_->close();
}

//TODO parameter did not get in
void FFC4PDriver::GetParameters(ros::NodeHandle& nh){
	nh.param<bool>("show_image",this->module_config_.show_img);
	nh.param<int32_t>("fps",this->module_config_.fps);
	nh.param<int32_t>("resolution",this->module_config_.resolution);
	nh.param<int32_t>("expose_time_us",this->module_config_.expose_time_us);
	nh.param<int32_t>("iso",this->module_config_.iso);
	nh.param<int32_t>("image_info",this->module_config_.show_img_info);
	nh.param<bool>("auto_awb", this->module_config_.auto_awb);
	nh.param<int32_t>("awb_value", this->module_config_.awb_value);
	nh.param<bool>("ros_defined_freq", this->module_config_.ros_defined_freq);
	switch (this->module_config_.resolution){
		case 720:{
			this->resolution_ = dai::ColorCameraProperties::SensorResolution::THE_720_P;
			break;
		}
		case 800:{
			this->resolution_ = dai::ColorCameraProperties::SensorResolution::THE_800_P;
			break;
		}
		default:{
			ROS_WARN("Unsupport resolution%d, setting to default 720p",this->module_config_.resolution);
			this->resolution_ = dai::ColorCameraProperties::SensorResolution::THE_720_P;
			break;
		}
	}
	// ROS_INFO("Parameter Setting List");
	// ROS_INFO("Start with Image viewer: %d",this->module_config_.show_img);
	// ROS_INFO("FPS: %d",this->module_config_.show_img);
	// ROS_INFO("Resolution: %d", this->module_config_.resolution);
	// ROS_INFO("")
	// ROS_INFO("Image info: %d", this->module_config_.show_img_info)

}

int32_t FFC4PDriver::InitPipeline(){
	this->pipeline_ = std::make_shared<dai::Pipeline>();
	if(this->pipeline_ ==nullptr){
		ROS_ERROR("pipline init failed\n");
		return  -1;
	}
	this->pipeline_->setXLinkChunkSize(0);
	// std::list<std::shared_ptr<dai::node::ColorCamera>> rgb_cam_list;
	for(int i = 0 ; i< this->CameraList.size(); i ++){
		auto rgb_cam = this->pipeline_->create<dai::node::ColorCamera>();

		rgb_cam->setResolution(this->resolution_);
		rgb_cam->setInterleaved(false);
		rgb_cam->setFps(this->module_config_.fps);
		rgb_cam->initialControl.setManualExposure(this->module_config_.expose_time_us,this->module_config_.iso);

		if(!this->module_config_.auto_awb){
			rgb_cam->initialControl.setManualWhiteBalance(this->module_config_.awb_value);
		}

		if(CameraList[i].is_master){
			printf("set %s as master camera\n",CameraList[i].stream_name.c_str());
			rgb_cam->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::OUTPUT);
		} else {
			printf("set %s as slave camera\n",CameraList[i].stream_name.c_str());
			rgb_cam->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);	
		}
		rgb_cam->setBoardSocket(CameraList[i].socket);

		auto xout_rgb = this->pipeline_->create<dai::node::XLinkOut>();
		if(xout_rgb ==nullptr){
			ROS_ERROR("xout link falied\n");
		}
    xout_rgb->setStreamName(CameraList[i].stream_name);
		ROS_INFO("Set stream name:%s\n",CameraList[i].stream_name.c_str());
		
    rgb_cam->video.link(xout_rgb->input);
		// rgb_cam_list.push_back(rgb_cam);
	}
	this->pipeline_is_init_ = 1;
	if(this->device_ != nullptr && this->device_is_detected_ ){
		this->device_->startPipeline(*this->pipeline_);
		return 0;
	}
	ROS_ERROR("Device is not init\n");
	return -2;
}

int32_t FFC4PDriver::SetAllCameraSychron(){
	if(this->device_is_detected_ == 0 || this->pipeline_ ==nullptr){
		ROS_ERROR("Device is not detected or pipeline is not initiated\n");
		return -1;
	}
	dai::Device::Config pipeline_config = this->pipeline_->getDeviceConfig();
	pipeline_config.board.gpio[6] = dai::BoardConfig::GPIO(dai::BoardConfig::GPIO::Direction::OUTPUT, 
		dai::BoardConfig::GPIO::Level::HIGH);
	return 0;
}

int32_t FFC4PDriver::SetVedioOutputQueue(){
	for(int i = 0; i < this->CameraList.size(); i++){
		auto rgb_queue = this->device_->getOutputQueue(this->CameraList[i].stream_name, 1, false);
		if(rgb_queue == nullptr){
			ROS_ERROR("Get video queue failed\n");
			return -1 ;
		} else {
			ROS_INFO("Get Out put queue %s success\n",this->CameraList[i].stream_name.c_str());
		}
		this->image_queue_.push_back(ImageNode(rgb_queue,this->CameraList[i].stream_name));
		ROS_INFO("queue back push %s success\n",this->CameraList[i].stream_name.c_str());
	}
	return 0;
}

void FFC4PDriver::StartVideoStream(){
	for(auto& i : this->image_queue_){
		std::stringstream topic;
		topic << "/oak_ffc_4p/image_" << i.topic;
		i.ros_publisher = this->ros_node_->advertise<sensor_msgs::Image>(topic.str(),1);
		ROS_INFO("Image topic %s publisher created",i.topic.c_str());
	}
	ROS_DEBUG("ros publisher established");

	if(this->module_config_.ros_defined_freq){
		printf("Use timer\n");
		this->thread_timer_  = this->ros_node_->createTimer(ros::Duration(1/this->module_config_.fps),&FFC4PDriver::RosGrabImgThread, this);
	} else{
		printf("Use thread\n");
		this->grab_thread_ = std::thread(&FFC4PDriver::GrabImgThread,this);
	}
	ROS_INFO("Start streaming\n");
	return;
}

void FFC4PDriver::RosGrabImgThread(const ros::TimerEvent &event){
	GrabImgThread();
}


//TODO:: fps counter, image show 
void FFC4PDriver::GrabImgThread(){
	cv_bridge::CvImage cv_img;
	auto host_ros_now_time = ros::Time::now();
	cv_img.header.stamp = host_ros_now_time;
	cv_img.header.frame_id = "depth ai";
	cv_img.encoding = "bgr8";
	for(auto & queue_node : this->image_queue_){
		auto video_frame = queue_node.data_output_q->tryGet<dai::ImgFrame>();
		if(video_frame != nullptr){
			printf("%s Get image\n",queue_node.topic.c_str());
			queue_node.image = video_frame->getCvFrame();
			queue_node.cap_time_stamp =  video_frame->getTimestamp();
			cv_img.image = queue_node.image;
			queue_node.ros_publisher.publish(cv_img.toImageMsg());
		} else {
			ROS_WARN("Get %s frame failed\n",queue_node.topic.c_str());
		}
	}
	if(this->module_config_.show_img){
		auto host_chrono_now_time = dai::Clock::now();
		for(auto & image_node : image_queue_){
			if(this->module_config_.show_img_info){
			//TODO image show infomation
			} else {
				//TODO image show
				cv::imshow(image_node.topic,image_node.image);
				cv::waitKey(1);
			}	
		}
	}
}

}