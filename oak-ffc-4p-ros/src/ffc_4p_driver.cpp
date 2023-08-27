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
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <depthai/utility/Clock.hpp>
#include <chrono>
#include <csignal>

#define SECOND 1000000
#define IMAGE_WIDTH 1280
#define FPS_BIAS 0

namespace OAKCAM{
FFC4PDriver::FFC4PDriver(std::shared_ptr<ros::NodeHandle>& nh){
	if(nh ==nullptr){
		ROS_ERROR("Init with a invalid Nodehandler");
		return;
	}
	cv::setNumThreads(1);
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
	nh.getParam("show_img",this->module_config_.show_img);
	nh.getParam("fps",this->module_config_.fps);
	nh.getParam("resolution",this->module_config_.resolution);
	nh.getParam("auto_expose",this->module_config_.auto_expose);
	nh.getParam("expose_time_us",this->module_config_.expose_time_us);
	nh.getParam("iso",this->module_config_.iso);
	nh.getParam("image_info",this->module_config_.show_img_info);
	nh.getParam("auto_awb", this->module_config_.auto_awb);
	nh.getParam("awb_value", this->module_config_.awb_value);
	nh.getParam("ros_defined_freq", this->module_config_.ros_defined_freq);
	nh.getParam("calibration_mode", this->module_config_.calibration_mode);
	nh.getParam("compresse_assemble_image", this->module_config_.compresse_assemble_image);
	
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
		rgb_cam->setFps(this->module_config_.fps+FPS_BIAS);
		if(this->module_config_.auto_expose){
			rgb_cam->initialControl.setAutoExposureEnable();
		} else {
			rgb_cam->initialControl.setManualExposure(this->module_config_.expose_time_us,this->module_config_.iso);
		}

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
		if(this->module_config_.calibration_mode){
			topic << "/oak_ffc_4p/image_" << i.topic <<"/compressed";
			i.ros_publisher = this->ros_node_->advertise<sensor_msgs::CompressedImage>(topic.str(),1);
		} else {
			topic << "/oak_ffc_4p/image_" << i.topic;
			i.ros_publisher = this->ros_node_->advertise<sensor_msgs::Image>(topic.str(),1);
		}
		
		ROS_INFO("Image topic %s publisher created",i.topic.c_str());
	}
	ROS_DEBUG("ros publisher established");

	this->expose_time_publisher_ = this->ros_node_->advertise<std_msgs::Int32>("/oak_ffc_4p/expose_time_us",1);
	if(this->module_config_.compresse_assemble_image){
		this->assemble_image_publisher_ = this->ros_node_->advertise<sensor_msgs::CompressedImage>("/oak_ffc_4p/assemble_image/compressed",1);
	} else {
		this->assemble_image_publisher_ = this->ros_node_->advertise<sensor_msgs::Image>("/oak_ffc_4p/assemble_image",1);
	}
	if(this->module_config_.ros_defined_freq){
		printf("Use timer\n");
		this->thread_timer_  = this->ros_node_->createTimer(ros::Duration(1/this->module_config_.fps*2),&FFC4PDriver::RosGrabImgThread, this);
	} else{
		printf("Use std thread\n");
		this->grab_thread_ = std::thread(&FFC4PDriver::StdGrabImgThread,this);
	}
	ROS_INFO("Start streaming\n");
	return;
}

void FFC4PDriver::RosGrabImgThread(const ros::TimerEvent &event){
	GrabImg();
}

void FFC4PDriver::StdGrabImgThread(){
	
	while(this->is_run_){
		GrabImg();
		usleep(SECOND/(2.0f*this->module_config_.fps));
	}
	ROS_INFO("Stop grab tread\n");
}

void FFC4PDriver::GrabImg(){
	static cv_bridge::CvImage cv_img, assemble_cv_img;
	static std_msgs::Int32 expose_time_msg;
	static cv::Mat assemble_cv_mat = cv::Mat::zeros(720,5120,CV_8UC3);
	auto host_ros_now_time = ros::Time::now();
	
	assemble_cv_img.header.stamp = host_ros_now_time;
	assemble_cv_img.header.frame_id = "depth ai";
	assemble_cv_img.encoding = "bgr8";
	assemble_cv_img.image = assemble_cv_mat;

	cv_img.header.stamp = host_ros_now_time;
	cv_img.header.frame_id = "depth ai";
	cv_img.encoding = "bgr8";

	expose_time_msg.data = this->module_config_.expose_time_us;

	auto host_time_now = dai::Clock::now();
	int colow_position = 0;
	int image_conter=0;

	for(auto && queue_node : this->image_queue_){
		auto video_frame = queue_node.data_output_q->tryGet<dai::ImgFrame>();
		if(video_frame != nullptr){
			queue_node.image = video_frame->getCvFrame();
			queue_node.cap_time_stamp =  video_frame->getTimestamp();
			image_conter++;
		} else {
			// ROS_WARN("Get %s frame failed\n",queue_node.topic.c_str());
		}
	}
	//calibration mode publish four compressed image and raw assemble

	if(image_conter == 4){//all cameras get images 
		if(this->module_config_.calibration_mode){
			for(auto && queue_node : this->image_queue_){
				cv_img.image = queue_node.image;
				queue_node.ros_publisher.publish(cv_img.toCompressedImageMsg());
			}
		} else {
			for(auto && queue_node : this->image_queue_){
				queue_node.image.copyTo(assemble_cv_img.image(cv::Rect(colow_position,0,1280,720)));
				colow_position += IMAGE_WIDTH;
			}
			if(this->module_config_.compresse_assemble_image){
				assemble_image_publisher_.publish(assemble_cv_img.toCompressedImageMsg());
			} else {
				assemble_image_publisher_.publish(assemble_cv_img.toImageMsg());
			}

		}
	}
	this->expose_time_publisher_.publish(expose_time_msg);
	if(this->module_config_.show_img){
		for(auto & image_node : image_queue_){
			this->ShowImg(image_node,host_time_now);
		}
	}
}

//TODO fps counter
void FFC4PDriver::ShowImg(ImageNode & image_node, std::chrono::_V2::steady_clock::time_point& time_now){
	if(image_node.image.empty()){
		// ROS_ERROR("Image empty\n");
		return;
	} else {
		if(!this->module_config_.show_img_info){
				// ROS_DEBUG("Show pure image\n");
				cv::imshow(image_node.topic.c_str(),image_node.image);
				cv::waitKey(1);
			} else {
				// printf("Show info image\n");
				double clearness = Clearness(image_node.image);
				uint32_t latency_us = std::chrono::duration_cast<std::chrono::microseconds>(time_now - image_node.cap_time_stamp).count();
				std::stringstream info;
				info << image_node.topic << "clearness: = " << clearness <<"    image_delay ms:=" << (latency_us/1000);
				cv::putText(image_node.image, info.str(), cv::Point(10, 30), 
					cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 0));
				cv::imshow(image_node.topic,image_node.image);
				cv::waitKey(1);
			}
	}
	return;
}

double Clearness(cv::Mat &img){
  //Clearness for focus
	if(img.empty()){
		// printf("img is empty");
		return 0.0f;
	} else {
		cv::Mat gray, imgSobel;
		cv::Rect2d roi(img.cols/3, img.rows/3, img.cols/3, img.rows/3);
		cv::rectangle(img, roi, cv::Scalar(255, 0, 0), 1);
		cv::cvtColor(img(roi), gray, cv::COLOR_BGR2GRAY);
		cv::Sobel(gray, imgSobel, CV_16U, 1, 1);
		return cv::mean(imgSobel)[0];
	}
}

}