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

namespace OAKCAM{

FFC4PDriver::FFC4PDriver(){
  ROS_INFO("FFC 4P Device Detecting\n");
  auto deviceInfoVec = dai::Device::getAllAvailableDevices();
  const auto usbSpeed = dai::UsbSpeed::SUPER;
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
}

FFC4PDriver::~FFC4PDriver(){
	this->device_->close();
}

int32_t FFC4PDriver::InitPipeline(){
	this->pipeline_ = std::make_shared<dai::Pipeline>();
	if(this->pipeline_ ==nullptr){
		ROS_ERROR("pipline init failed\n");
		return  -1;
	}
	// std::list<std::shared_ptr<dai::node::ColorCamera>> rgb_cam_list;
	for(int i = 0 ; i< this->CameraList.size(); i ++){
		auto rgb_cam = this->pipeline_->create<dai::node::ColorCamera>();

		rgb_cam->setResolution(CameraList[i].resolution);
		rgb_cam->setInterleaved(false);
		rgb_cam->setFps(30);
		rgb_cam->initialControl.setManualExposure(1000,800);
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
	dai::Device::Config pipeline_config = this->pipeline_->getDeviceConfig();
	pipeline_config.board.gpio[6] = dai::BoardConfig::GPIO(dai::BoardConfig::GPIO::Direction::OUTPUT, 
	dai::BoardConfig::GPIO::Level::HIGH);
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
	// dai::Device::Config pipeline_config = this->pipeline_->getDeviceConfig();
	// pipeline_config.board.gpio[6] = dai::BoardConfig::GPIO(dai::BoardConfig::GPIO::Direction::OUTPUT, 
	// 	dai::BoardConfig::GPIO::Level::HIGH);
	return 0;
}

int32_t FFC4PDriver::SetVedioOutputQueue(){
	for(int i = 0; i < this->CameraList.size(); i++){
		auto rgb_queue = this->device_->getOutputQueue(this->CameraList[i].stream_name, 2, false);
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

void FFC4PDriver::GrabImgThread(){
	int i =0;
	for(auto & queue_node : this->image_queue_){
		auto video_frame = queue_node.data_output_q->tryGet<dai::ImgFrame>();
		if(video_frame != nullptr){
			cv::imshow(queue_node.topic,video_frame->getCvFrame());
			cv::waitKey(10);
		} else {
			// printf("Get %s frame failed\n",queue_node.topic.c_str());
		}
	}
}

}