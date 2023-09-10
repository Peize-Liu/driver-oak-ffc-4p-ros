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

int main(int argc, char** argv){
	ros::init(argc, argv, "oakcam_ffc_4p_ros");
    auto cam_node = std::make_shared<ros::NodeHandle>("oakcam_ffc_4p_ros") ;
	OAKCAM::FFC4PDriver cam_driver(cam_node);
	int32_t ret = 0 ;
	ret = cam_driver.InitPipeline();
	if(ret != 0){
		printf("init pipeline failed\n");
		return -1 ;
	}
	ret = cam_driver.SetAllCameraSychron();
	if(ret != 0){
		printf("Set Cam sychron failed\n");
		return -2 ;
	}
	ret = cam_driver.SetVedioOutputQueue();
	if(ret !=0){
		printf("Set video queue failed\n");
		return -3;
	}
    cam_driver.StartVideoStream();
    ros::spin();
    cam_driver.StopVideoStream();
    usleep(500);
    return 0 ;
}

#if 0 
std::shared_ptr<dai::Pipeline> createPipeline() {
    // Start defining a pipeline
    auto pipeline = std::make_shared<dai::Pipeline>();
    // Define a source - color camera
    auto camRgb = pipeline->create<dai::node::ColorCamera>();

    // camRgb->setPreviewSize(300, 300);
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);

    // Create output
    auto xoutRgb = pipeline->create<dai::node::XLinkOut>();
    xoutRgb->setStreamName("rgb");
    camRgb->preview.link(xoutRgb->input);

    auto camRgb_b = pipeline->create<dai::node::ColorCamera>();

    // camRgb->setPreviewSize(300, 300);
    camRgb_b->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    camRgb_b->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb_b->setInterleaved(false);
    auto xoutRgb_b = pipeline->create<dai::node::XLinkOut>();
    xoutRgb_b->setStreamName("rgb_b");
    camRgb_b->preview.link(xoutRgb_b->input);

    return pipeline;
}

int main(int argc, char **argv) {
    auto deviceInfoVec = dai::Device::getAllAvailableDevices();
    const auto usbSpeed = dai::UsbSpeed::SUPER;
    auto openVinoVersion = dai::OpenVINO::Version::VERSION_2021_4;

    std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> qRgbMap;
    std::vector<std::shared_ptr<dai::Device>> devices;

    for(auto& deviceInfo : deviceInfoVec) {
        auto device = std::make_shared<dai::Device>(openVinoVersion, deviceInfo, usbSpeed);
        devices.push_back(device);
        std::cout << "===Connected to " << deviceInfo.getMxId() << std::endl;
        auto mxId = device->getMxId();
        auto cameras = device->getConnectedCameras();
        auto usbSpeed = device->getUsbSpeed();
        auto eepromData = device->readCalibration2().getEepromData();
        std::cout << "   >>> MXID:" << mxId << std::endl;
        std::cout << "   >>> Num of cameras:" << cameras.size() << std::endl;
        std::cout << "   >>> USB speed:" << usbSpeed << std::endl;
        if(eepromData.boardName != "") {
            std::cout << "   >>> Board name:" << eepromData.boardName << std::endl;
        }
        if(eepromData.productName != "") {
            std::cout << "   >>> Product name:" << eepromData.productName << std::endl;
        }
        auto pipeline = createPipeline();
        device->startPipeline(*pipeline);

        auto qRgb = device->getOutputQueue("rgb", 4, false);
        auto qRgb_b = device->getOutputQueue("rgb_b", 4, false);
        qRgbMap.insert({"cam_a", qRgb});
        qRgbMap.insert({"cam_b", qRgb_b});
    }
    while(true){
        auto rgb_img_a = qRgbMap.find("cam_a");
        auto rgb_img_b = qRgbMap.find("cam_b");
        auto img = rgb_img_a->second->tryGet<dai::ImgFrame>();
        auto img_b = rgb_img_b->second->tryGet<dai::ImgFrame>();
        if(img != nullptr){
            cv::imshow("test",img->getCvFrame());
            cv::imshow("test_2",img_b->getCvFrame());
            // cv::imshow("test_b",img_b->getCvFrame());
            cv::waitKey(1);
        }
    }

}
#endif 
// #include <iostream>

// Includes common necessary includes for development using depthai library


#if 0
int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto CAM_A = pipeline.create<dai::node::ColorCamera>();
    auto CAM_A_Xout = pipeline.create<dai::node::XLinkOut>();
    CAM_A_Xout->setStreamName("CAM_A");
    // Properties
    CAM_A->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    CAM_A->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
    CAM_A_Xout->input.setBlocking(false);
    CAM_A_Xout->input.setQueueSize(1);
    CAM_A->video.link(CAM_A_Xout->input);

        // Define sources and outputs
    auto CAM_B = pipeline.create<dai::node::ColorCamera>();
    auto CAM_B_Xout = pipeline.create<dai::node::XLinkOut>();
    CAM_B_Xout->setStreamName("CAM_B");
    // Properties
    CAM_B->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    CAM_B->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
    CAM_B_Xout->input.setBlocking(false);
    CAM_B_Xout->input.setQueueSize(1);
    CAM_B->video.link(CAM_B_Xout->input);

        // Define sources and outputs
    auto CAM_C = pipeline.create<dai::node::ColorCamera>();
    auto CAM_C_Xout = pipeline.create<dai::node::XLinkOut>();
    CAM_C_Xout->setStreamName("CAM_C");
    // Properties
    CAM_C->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    CAM_C->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
    CAM_C_Xout->input.setBlocking(false);
    CAM_C_Xout->input.setQueueSize(1);
    CAM_C->video.link(CAM_C_Xout->input);

        // Define sources and outputs
    auto CAM_D = pipeline.create<dai::node::ColorCamera>();
    auto CAM_D_Xout = pipeline.create<dai::node::XLinkOut>();
    CAM_D_Xout->setStreamName("CAM_D");
    // Properties
    CAM_D->setBoardSocket(dai::CameraBoardSocket::CAM_D);
    CAM_D->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
    CAM_D_Xout->input.setBlocking(false);
    CAM_D_Xout->input.setQueueSize(1);
    CAM_D->video.link(CAM_D_Xout->input);



    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues will be used to get the grayscale frames from the outputs defined above
    auto video_a = device.getOutputQueue("CAM_A");
    auto video_b = device.getOutputQueue("CAM_B");
    auto video_c = device.getOutputQueue("CAM_C");
    auto video_d = device.getOutputQueue("CAM_D");


    while(true) {
        // Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
    auto video_cam_a_frame = video_a->tryGet<dai::ImgFrame>();
    auto video_cam_b_frame = video_b->tryGet<dai::ImgFrame>();
    auto video_cam_c_frame = video_c->tryGet<dai::ImgFrame>();
    auto video_cam_d_frame = video_d->tryGet<dai::ImgFrame>();
    if(video_cam_a_frame != nullptr){
        printf("show image\n");
        // std::vector<cv::Mat>images[4]={,video_cam_b_frame->getCvFrame(),video_cam_c_frame->getCvFrame(),video_cam_d_frame->getCvFrame()};
        cv::imshow("CAM_A",video_cam_a_frame->getCvFrame());
        cv::imshow("CAM_B",video_cam_b_frame->getCvFrame());
        cv::imshow("CAM_V",video_cam_c_frame->getCvFrame());
        cv::imshow("CAM_D",video_cam_d_frame->getCvFrame());
        printf("xxxx\n");
    }
    cv::waitKey(1);    // auto video_cam_b_frame = video_b->get<dai::ImgFrame>();
    // auto video_cam_c_frame = video_c->get<dai::ImgFrame>();
    // auto video_cam_d_frame = video_d->get<dai::ImgFrame>();
        // cv::imshow("CAM_B",video_cam_b_frame->getCvFrame());      
        // cv::imshow("CAM_C",);      
        // cv::imshow("CAM_D",);
    }
    return 0;
}
#endif
