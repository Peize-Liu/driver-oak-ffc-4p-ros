# Introduction
This package is for oak_ffc_4P camera(OV9782) to publish ros topics with 

**calibration mode**:

​	/oak_ffc_4p/image_CAM_A

​	/oak_ffc_4p/image_CAM_B

​	/oak_ffc_4p/image_CAM_C

​	/oak_ffc_4p/image_CAM_D

**fully synchronized mode**:

​	/oak_ffc_4p/assemble_image

## how to install depthai-core-v2.21.2
`mkdir build`
`cd build`
`cmake ../`
`make install`

then modify oak_ffc_4p's CMakeLists.txt to specify the local
## Usage

launch file paramters:

```yaml
    <arg name="show_img" default="false" /> # show image 
    <arg name="fps" default = "25" />
    <arg name="resolution" default = "720"/>
    <arg name="auto_expose" default = "false"/>
    <arg name="expose_time_us" default = "15000"/>
    <arg name="iso" default = "200"/>
    <arg name="image_info" default= "true"/>  #show image info like clearness and latency
    <arg name="auto_awb" default = "true"/>
    <arg name="awb_value" default = "4000"/>
    <arg name="ros_defined_freq" default = "false"/>
    <arg name="calibration_mode" default = "false"/> # false in fully synchronized mode / true for calibration mode
    <arg name="compresse_assemble_image" default = "false"/> #compress assemble image or not
```



set <arg name="show_img" default="true" />  you can see image

![image-20230514221212232](./image/image-20230514221212232.png)



further more set  **image_info** to true you can see clearness and transportation latency.

![image-20230514220918697](./image/image-20230514220918697.png)





## Note

This is a third-party package, it is not maintained by the manufacturer. 

## LICENSE
LGPL-V3