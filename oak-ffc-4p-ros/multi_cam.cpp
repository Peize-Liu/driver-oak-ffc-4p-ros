#include <chrono>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

std::shared_ptr<dai::Pipeline> createPipeline() {
    // Start defining a pipeline
    auto pipeline = std::make_shared<dai::Pipeline>();
    // Define a source - color camera
    auto camRgb = pipeline->create<dai::node::ColorCamera>();

    // camRgb->setPreviewSize(300, 300);
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
    camRgb->setInterleaved(false);

    // Create output
    auto xoutRgb = pipeline->create<dai::node::XLinkOut>();
    xoutRgb->setStreamName("rgb");
    camRgb->video.link(xoutRgb->input);

    auto camRgb_b = pipeline->create<dai::node::ColorCamera>();

    // camRgb->setPreviewSize(300, 300);
    camRgb_b->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    camRgb_b->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
    camRgb_b->setInterleaved(false);
    auto xoutRgb_b = pipeline->create<dai::node::XLinkOut>();
    xoutRgb_b->setStreamName("rgb_b");
    camRgb_b->video.link(xoutRgb_b->input);

    auto camRgb_c = pipeline->create<dai::node::ColorCamera>();
    camRgb_c->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    camRgb_c->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
    camRgb_c->setInterleaved(false);
    auto xoutRgb_c = pipeline->create<dai::node::XLinkOut>();
    xoutRgb_c->setStreamName("rgb_c");
    camRgb_c->video.link(xoutRgb_c->input);

    auto camRgb_d = pipeline->create<dai::node::ColorCamera>();
    camRgb_d->setBoardSocket(dai::CameraBoardSocket::CAM_D);
    camRgb_d->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
    camRgb_d->setInterleaved(false);
    auto xoutRgb_d = pipeline->create<dai::node::XLinkOut>();
    xoutRgb_d->setStreamName("rgb_d");
    camRgb_d->video.link(xoutRgb_d->input);

    return pipeline;
}

int main(int argc, char** argv) {
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
        device->config.board.gpio[6] = dai::BoardConfig::GPIO(dai::BoardConfig::GPIO::OUTPUT,dai::BoardConfig::GPIO::Level::HIGH);
        device->config
        auto pipeline = createPipeline();
        device->startPipeline(*pipeline);

        auto qRgb = device->getOutputQueue("rgb", 2, false);
        qRgbMap.insert({"cam_a", qRgb});

        auto qRgb_b = device->getOutputQueue("rgb_b", 2, false);
        qRgbMap.insert({"cam_b", qRgb_b});

        auto qRgb_c = device->getOutputQueue("rgb_c", 2, false);
        qRgbMap.insert({"cam_c", qRgb_c});

        auto qRgb_d = device->getOutputQueue("rgb_d", 2, false);
        qRgbMap.insert({"cam_d", qRgb_d});
        
    }
    while(true) {
        for(auto& element : qRgbMap) {
            auto qRgb = element.second;
            auto streamName = element.first;
            auto inRgb = qRgb->tryGet<dai::ImgFrame>();
            if(inRgb != nullptr) {
                cv::imshow(streamName, inRgb->getCvFrame());
            }
        }
        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
