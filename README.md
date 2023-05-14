# Introduction
This package is for oak_ffc_4P camera(OV9782) to publish ros topics with  

/oak_ffc_4p/image_CAM_A

/oak_ffc_4p/image_CAM_B

/oak_ffc_4p/image_CAM_C

/oak_ffc_4p/image_CAM_D

## Usage

launch file paramters:

```yaml
<arg name="show_img" default="false" /> // show image with cv img shower
<arg name="fps" default = "30" /> // camera fps, upto 30 fps
<arg name="resolution" default = "720"/> //Camera resolution for OV9872 only 720 or 800
<arg name="expose_time_us" default = "4000"/> //set expose time
<arg name="auto_expose" default = "true"/> //set auto expose
<arg name="iso" default = "400"/> // set iso
<arg name="image_info" default= "true"/> //set whether show clearness info and latency info or not
<arg name="auto_awb" default = "true"/> //set auto wight balence or not 
<arg name="awb_value" default = "4000"/>//set wight balence value
<arg name="ros_defined_freq" default = "false"/>// set use std::thread or ros timer to grab image(std::thread cpu loading is lower but fps will not be stable to setting one)
```



set <arg name="show_img" default="true" />  you can see image

![image-20230514221212232](./image/image-20230514221212232.png)



further more set  **image_info** to true you can see clearness and transportation latency.

![image-20230514220918697](./image/image-20230514220918697.png)





## Note

This is a third-party package, it is not maintained by the manufacturer. 

## LICENSE
LGPL-V3