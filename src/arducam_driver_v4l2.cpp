#include "arducam_driver_v4l2.hpp"

cv::Mat& convert(cv::Mat& raw_imag, int& rows);
void setExposureGain(int exp, int gain);
double clearness(cv::Mat & img) ;

int32_t OAKFFC4PSDK::CameraNode::Init(dai::Pipeline & pipeline, dai::CameraBoardSocket board_socket,
  dai::MonoCameraProperties::SensorResolution resolution, uint32_t buffer_size, bool block_mode){
  // if(camera_type_ == Color){
  //   ROS_ERROR("Camera Structure as Color Camera but init with Mono camera resolution\n"); 
  //   is_init_ = 0;
  //   return -1;
  // }
  // mono_resolution_ = resolution;
  // board_socket_ = board_socket;
  // mono_cam_ptr_= pipeline.create<dai::node::MonoCamera>();
  // xlink_out_ptr_= pipeline.create<dai::node::XLinkOut>();
  // xlink_out_ptr_->setStreamName(stream_name);
  // mono_cam_ptr_->setBoardSocket(board_socket);
  // mono_cam_ptr_->setResolution(resolution);

  // printf("Camera set stream name: %s  board_socket:%d\n",stream_name.c_str(), int32_t(board_socket));
  // xlink_out_ptr_->input.setQueueSize(buffer_size);
  // xlink_out_ptr_->input.setBlocking(block_mode);
  // mono_cam_ptr_->out.link(xlink_out_ptr_->input);
  // output_queue_ptr_ = device.getOutputQueue(stream_name);
  // if(output_queue_ptr_ == nullptr){
  //   printf("Init camera node failed, cannot get output queue\n");
  //   return -1;
  // }
  // is_init_ = 1;
  // return 0;
}

int32_t OAKFFC4PSDK::CameraNode::Init(dai::Pipeline & pipeline, dai::CameraBoardSocket board_socket,
  dai::ColorCameraProperties::SensorResolution resolution, uint32_t buffer_size, bool block_mode){
  if(camera_type_ == Mono){
    ROS_ERROR("Camera Structure as Mono Camera but init with Color camera resolution\n"); 
    is_init_ = -1;
    return -1;
  }
  color_resolution_ = resolution;
  board_socket_ = board_socket;

  color_cam_ptr_=pipeline.create<dai::node::ColorCamera>();
  if(color_cam_ptr_ ==nullptr){
    printf("camera pipeline init failed\n");
    return -2;
  }
  xlink_out_ptr_= pipeline.create<dai::node::XLinkOut>();
  if(xlink_out_ptr_ == nullptr){
    printf("xlink init failed\n");
    return -3;
  }

  xlink_out_ptr_->setStreamName(stream_name);
  color_cam_ptr_->setBoardSocket(board_socket);
  color_cam_ptr_->setResolution(resolution);
  printf("Camera set stream name: %s  board_socket:%d\n",stream_name.c_str(), int32_t(board_socket));
  xlink_out_ptr_->input.setQueueSize(buffer_size);
  xlink_out_ptr_->input.setBlocking(block_mode);
  color_cam_ptr_->video.link(xlink_out_ptr_->input);
  // color_cam_ptr_->out
  is_init_ = 1;
  return 0;
}


// int32_t OAKFFC4PSDK::CameraNode::SetBuffer(dai::Device & device, unsigned int buffer_size, bool block_mode){
//   if(!is_init_){
//     return -1;
//   }
//   output_queue_ptr_ = device.getOutputQueue(stream_name);
//   return 0;
// }

int32_t OAKFFC4PSDK::CameraNode::GrabImage(cv::Mat& image){
  if(!is_init_ || father_device_ptr_ ==nullptr){
    printf("fatuer device init falied\n");
    return -1;
  }
  if(output_queue_ptr_ == nullptr){
    output_queue_ptr_ = father_device_ptr_->getOutputQueue(xlink_out_ptr_->getStreamName());
    if(output_queue_ptr_ == nullptr){
      printf("Grap img from camera node failed, cannot get output queue\n");
      return -1;
    }
  }
  auto get_frame = output_queue_ptr_->tryGet<dai::ImgFrame>();
  if(get_frame != nullptr){
    printf("grab inmage\n");
    image = get_frame->getCvFrame();
    return 0;
  }
  return -2 ;
}

OAKFFC4PSDK::OAKFFC4PDriver::OAKFFC4PDriver(){
  cv::setNumThreads(1);
}

OAKFFC4PSDK::OAKFFC4PDriver::~OAKFFC4PDriver(){

}

int32_t OAKFFC4PSDK::OAKFFC4PDriver::Init(ros::NodeHandle & nh) {
//Load parameters
  // nh.param<bool>("raw8", config_.raw8);
  // nh.param<bool>("publish_splited", config_.publish_splited);
  // nh.param<bool>("print_clearness", config_.print_clearness);
  // nh.param<bool>("show", config_.show);
  // nh.param<bool>("sync", config_.is_sync);
  // nh.param<int>("fps", config_.fps);
  // nh.param<int>("width", config_.width);
  // nh.param<int>("height", config_.height);
  // nh.param<int>("cap_device", config_.cap_device);
  // nh.param<int>("camera_num", config_.camera_num);
  // nh.param<int>("exposure", config_.exposure);
  // nh.param<int>("gain", config_.gain);
  // nh.param<int>("camera_num",config_.camera_amount);
  // nh.param("buffer_frame_number",config_.buffer_frame_num);
  // printf("Set fps:%d , set: buffer_frame_number:%d \n",config_.fps, config_.buffer_frame_num);
//Open OAK camera pipeline & config
  printf("xxxx\n");

#if 1
  for(int i=0 ; i<4; i++){
    if(i>5){
      ROS_ERROR("Driver only support upto 6 Cameras\n");
      return -1;
    }
    if(config_.is_color){
      CameraNode new_node(config_.name_vec[i],CameraNode::CameraType::Color);
      printf("color camera init\n");
      new_node.Init(pipeline_, dai::CameraBoardSocket(int(dai::CameraBoardSocket::CAM_A)+i),
        dai::ColorCameraProperties::SensorResolution::THE_720_P, 1, false );
      oak_cam_list_.push_back(new_node);
    } else {
      printf("mono camera init\n");
      CameraNode new_node(config_.name_vec[i],CameraNode::CameraType::Mono);
      new_node.Init(pipeline_,dai::CameraBoardSocket(int(dai::CameraBoardSocket::CAM_A)+i),
        dai::MonoCameraProperties::SensorResolution::THE_720_P, 1, false);
      oak_cam_list_.push_back(new_node);
    }
  }
  device_ptr_ = std::make_shared<dai::Device>(pipeline_);
  for(int i =0; i<4 ;i++){
    oak_cam_list_[i].SetFatherDevice(device_ptr_);
  }
  #endif
  auto deviceInfoVec = dai::Device::getAllAvailableDevices();
  const auto usbSpeed = dai::UsbSpeed::SUPER;
  auto openVinoVersion = dai::OpenVINO::Version::VERSION_2021_4;
  printf("xxxx1\n");
  for(auto& deviceInfo : deviceInfoVec) {
      std::cout << "===Connected to " << deviceInfo.getMxId() << std::endl;
      auto mxId = device_ptr_->getMxId();
      auto cameras = device_ptr_->getConnectedCameras();
      auto usbSpeed = device_ptr_->getUsbSpeed();
      auto eepromData = device_ptr_->readCalibration2().getEepromData();
      std::cout << "   >>> MXID:" << mxId << std::endl;
      std::cout << "   >>> Num of cameras:" << cameras.size() << std::endl;
      std::cout << "   >>> USB speed:" << usbSpeed << std::endl;
      if(eepromData.boardName != "") {
          std::cout << "   >>> Board name:" << eepromData.boardName << std::endl;
      }
      if(eepromData.productName != "") {
          std::cout << "   >>> Product name:" << eepromData.productName << std::endl;
      }
  }

  // Create publishers
  image_transport_ = new image_transport::ImageTransport(nh);
  pub_raw_ = image_transport_->advertise("/aok/image", 1);

  if (config_.publish_splited) {
      for (int i = 0; i < config_.camera_num; i++) {
          std::stringstream ss;
          ss << "/oak/image_" << i;
          pub_splited_.push_back(image_transport_->advertise(ss.str(), 1));
      }
  }

  //Start grab thread if success
  if (config_.is_sync) {
      grab_thread_ = std::thread(&OAKFFC4PDriver::grabThread, this);
  } else {
      //Software trigger
      grab_timer_ = nh.createTimer(ros::Duration(1.0/config_.fps), &OAKFFC4PDriver::grabRos, this);
  }
  printf("Image caputure start\n");
  init_flag_ = 0;

  return 0;

}


void OAKFFC4PSDK::OAKFFC4PDriver::grabRos(const ros::TimerEvent & event) {
    grab();
}

void OAKFFC4PSDK::OAKFFC4PDriver::grab() {
  if (frame_count_ == 0) {
      tstart_ = ros::Time::now();
  }

  // for(auto&i :oak_cam_list_){

  // }

  for(int i = 0; i < oak_cam_list_.size();i++){
    cv::Mat image;
    auto ret = oak_cam_list_[i].GrabImage(image);
    if(ret !=0){
     ROS_ERROR("Failed in getting image from %s\n",oak_cam_list_[i].stream_name.c_str());
     continue; 
    }
    cv_image_vec_[i] = image;
  }
  //publish
  auto ts = ros::Time::now();
  for(int i = 0 ; i < cv_image_vec_.size() ; i++){
    if(!cv_image_vec_[i].empty()){
      cv_bridge::CvImage cv_img;
      cv_img.header.stamp = ts;
      cv_img.header.frame_id = "oak";
      cv_img.image = cv_image_vec_[i];
      pub_splited_[i].publish(cv_img.toImageMsg());
    }
  }
  if (frame_count_ ==0){

  }
  frame_count_++;
  double tgrab = (ros::Time::now() - tstart_).toSec();
  ROS_INFO_THROTTLE(1.0, "[ArduCam] Total %d freq:%.1ffps", 
  frame_count_, frame_count_/tgrab);


  // memcpy((uint8_t*)local_frame_addr,(uint8_t*)buffer_vect_[new_image.index].image_buffer_addr,local_frame_size);
  // cv::Mat frame(config_.width,config_.height,CV_8U,local_frame_addr); //占用极高的问题在于没有进行拷贝

  // frame = convert(frame, config_.height); //占用及其高
  // if(ioctl(cam_fd_,VIDIOC_QBUF,&new_image) < 0){
  //   printf("return buffer failed\n");
  // }
  // return;
  // if (!frame.empty()) {
  //   cv_bridge::CvImage cv_img;
  //   cv_img.header.stamp = ts;
  //   cv_img.header.frame_id = "arducam";
  //   cv_img.encoding = "bgr8";
  //   cv_img.image = frame;
  //   cv::Mat show_image;
  //   if (config_.show && cam_shown_ == -1) {
  //     cv::resize(frame, show_image, 
  //         cv::Size(frame.cols / config_.camera_num, frame.rows / config_.camera_num));
  //   }
  //   //publish
  //   pub_raw_.publish(cv_img.toImageMsg());
  //   if (config_.publish_splited || config_.print_clearness || (cam_shown_ >= 0 && config_.show)) {
  //     if (config_.print_clearness) {
  //         printf("[ArduCam] clearness:");
  //     }
  //     for (int i = 0; i < config_.camera_num; i++) {
  //       if (config_.publish_splited || cam_shown_ == i || config_.print_clearness) {
  //           cv_img.image = frame(cv::Rect(i * frame.cols / config_.camera_num, 0, 
  //                   frame.cols / config_.camera_num, frame.rows));
  //           if (config_.publish_splited) {
  //               pub_splited_[i].publish(cv_img.toImageMsg());
  //           }
  //           if (config_.print_clearness) {
  //               printf("%d: %.1f%%\t", i, clearness(cv_img.image)*100);
  //           }
  //       }
  //       if (config_.show && cam_shown_ == i) {
  //           show_image = cv_img.image;
  //       }
  //     }
  //     if (config_.print_clearness) {
  //         printf("\n");
  //     }
  //   }

  //   if (config_.show) {
  //       printf("Show image\n");
  //       showImage(show_image);
  //   }

  //   if (frame_count_ == 0) {
  //       setExposureGain(config_.exposure, config_.gain);
  //   }
  //   frame_count_ ++;
  //   double tgrab = (ros::Time::now() - tstart_).toSec();
  //   ROS_INFO_THROTTLE(1.0, "[ArduCam] Total %d freq:%.1ffps", 
  //     frame_count_, frame_count_/tgrab);
  // } else {
  //   ROS_WARN("[ArduCam] Failed to grab a frame");
  // }

  // //returb buffer to kernel
  // if(ioctl(cam_fd_,VIDIOC_QBUF,&new_image) < 0){
  //   printf("return buffer failed\n");
  // }
}

void OAKFFC4PSDK::OAKFFC4PDriver::showImage(cv::Mat & show_image) {

  // char title[64] = {0};
  // if (cam_shown_ != -1) {
  //     //Show clearness on image
  //     double clear = clearness(show_image);
  //     sprintf(title, "Clearness %.1f%%", clear*100);
  //     cv::putText(show_image, title, cv::Point(10, 30), 
  //             cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 0));
  // }
  // sprintf(title, "Cam %d +/- to switch", cam_shown_);
  // cv::putText(show_image, title, cv::Point(10, 10), 
  //             cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 255));

  // cv::imshow("ArduCam", show_image);
  // int key = cv::waitKey(1);
  // if (key==61) {
  //   cam_shown_ = (cam_shown_ + 2)%(config_.camera_num + 1) - 1;
  // } else {
  //   if (key==45){
  //     cam_shown_ = cam_shown_%(config_.camera_num + 1) - 1;
  //   }
  // } 
}

void OAKFFC4PSDK::OAKFFC4PDriver::grabThread() {
  ROS_INFO("[OAKCAM Driver] Start to grab\n");
  tstart_ = ros::Time::now();
  while (ros::ok()) {
    grab();
    ros::Duration(0.1 / config_.fps).sleep();
  }
  return;
}

cv::Mat& convert(cv::Mat& raw_imag, int& rows) {
  // cv::Mat img = raw_imag.reshape(0, rows);
  // static cv::Mat encode_image;
  // struct timeval start_time, end_time;
  // gettimeofday(&start_time,NULL);
  // cv::cvtColor(img, encode_image, cv::COLOR_BayerRG2BGR);
  // gettimeofday(&end_time,NULL);
  // // printf("use time: usec:%ld\n",(end_time.tv_sec-start_time.tv_sec)*1000000 + (end_time.tv_usec-start_time.tv_usec));
  // return encode_image;
}

void setExposureGain(int exp, int gain) {
  // char cmd[64] = {0};
  // printf("Setting exposure to %d gain to %d by v4l2-ctrl", exp, gain);
  // sprintf(cmd, "/usr/bin/v4l2-ctl -c exposure=%d", exp);
  // auto ret = system(cmd);
  // sprintf(cmd, "/usr/bin/v4l2-ctl -c gain=%d", gain);
  // ret = system(cmd);
}

double clearness(cv::Mat & img) {
  // //Clearness for focus
  // cv::Mat gray, imgSobel;
  // cv::Rect2d roi(img.cols/3, img.rows/3, img.cols/3, img.rows/3);
  // cv::rectangle(img, roi, cv::Scalar(255, 0, 0), 1);
  // cv::cvtColor(img(roi), gray, cv::COLOR_BGR2GRAY);
  // cv::Sobel(gray, imgSobel, CV_16U, 1, 1);
  // return cv::mean(imgSobel)[0];
}