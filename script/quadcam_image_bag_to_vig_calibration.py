#!/usr/bin/env python3
#To extract specific image from bag file and then store the image with png format

#usage: python3 ./quadcam_image_extractor.py --input_bag /media/khalil/ssd_data/data_set/omnicam/omni_calibration_CAMA.bag  --topic /oak_ffc_4p/image_CAM_A/compressed\
                #--output_dir /home/khalil/workspace/tools/test \
                #--step 4 --num 200 --start 20
#only for oak_ffc_4p

#!/usr/bin/env python3
#to extract image with sepecific step from bag file
#usage: python3 quadcam_extract.py -i input.bag -s 10 -t 0.5
#only for oak_ffc_4p


import rosbag
from os.path import exists
from cv_bridge import CvBridge
import cv2 as cv
import tqdm
import numpy as np
import os
import zipfile

def generate_bagname(bag, comp=False):
    from pathlib import Path
    p = Path(bag)
    bagname = p.stem + "-split.bag"
    output_bag = p.parents[0].joinpath(bagname)
    return output_bag

def genereate_imagename(image_cont,output_path):
  image_name = output_path + "/images/"+ "{:05d}".format(image_cont) + ".png"
  # print("image name",image_name)
  return image_name

def zip_folder(folder_path, output_path):
    with zipfile.ZipFile(output_path, 'w', zipfile.ZIP_DEFLATED) as zipObj:
        # Iterate over all the files in directory tree
        for foldername, subfolders, filenames in os.walk(folder_path):
            for filename in filenames:
                # Create complete filepath of file in directory
                file_path = os.path.join(foldername, filename)
                # Get relative path of file
                rel_path = os.path.relpath(file_path, folder_path)
                # Add file to zip with a relative path
                zipObj.write(file_path, arcname=rel_path)

   
topic_list = [
    "/oak_ffc_4p/image_CAM_A",
    "/oak_ffc_4p/image_CAM_B",
    "/oak_ffc_4p/image_CAM_C",
    "/oak_ffc_4p/image_CAM_D",
    "/oak_ffc_4p/image_CAM_A/compressed",
    "/oak_ffc_4p/image_CAM_B/compressed",
    "/oak_ffc_4p/image_CAM_C/compressed",
    "/oak_ffc_4p/image_CAM_D/compressed",
    "/oak_ffc_4p/expose_time_us"]

topic_conter = {"/oak_ffc_4p/image_CAM_A":0,
                "/oak_ffc_4p/image_CAM_B":0,
                "/oak_ffc_4p/image_CAM_C":0,
                "/oak_ffc_4p/image_CAM_D":0,
                "/oak_ffc_4p/image_CAM_A/compressed":0,
                "/oak_ffc_4p/image_CAM_B/compressed":0,
                "/oak_ffc_4p/image_CAM_C/compressed":0,
                "/oak_ffc_4p/image_CAM_D/compressed":0}

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Split quadcam images')
    parser.add_argument("--input_bag","--input", type=str, help="input bag file")
    parser.add_argument("--topic","--topic", type=str, help="image topic to extract")
    parser.add_argument("--output_dir","--output_dir",type=str,help="output dir path")
    parser.add_argument("--num","--num",type=int,help="number of images to extract",default=-1)
    parser.add_argument("--start","--start", type=int, nargs="?", help="start time of the first image, default 0", default=0)
    parser.add_argument("--step", "--step", type=int, nargs="?", help="step for images, default 1", default=1)
    parser.add_argument("--expose_time_ms",type=float,help="expose time in ms",default=15)
    parser.add_argument("--undistort","--undistort",type=bool,help="undistort image",default=False)


    args = parser.parse_args()
    # output_bag = generate_bagname(args.input_bag)
    if not exists(args.input_bag):
      print(f"Input bag file {args.input_bag} does not exist")
      exit(1)
    if not exists(args.output_dir):
      print(f"Image save dir {args.output_dir} does not exist")
      exit(1)
    
    # open time-stamp file
    file = open(args.output_dir + "/times.txt","w")
    if file == None:
      print("open time-stamp file failed")
      exit(1)
    #create images folder
    if not exists(args.output_dir + "/images/"):
      print("create images folder")
      os.makedirs(args.output_dir + "/images/")
    bag = rosbag.Bag(args.input_bag)
    expose_time_ms = args.expose_time_ms
    extract_image_topic = args.topic
    total_num_imgs = bag.get_message_count(extract_image_topic)
    if total_num_imgs <= 0:
      print(f"Bag does not contain topic {extract_image_topic}")
      exit(1)
    else:
      print(f"Bag contains topic {extract_image_topic} with {total_num_imgs} images")
      if args.num > total_num_imgs:
        print(f"Bag contains topic {extract_image_topic} with {total_num_imgs} images, but you want to extract {args.num} images")
        extract_image_num = total_num_imgs
      else:
        extract_image_num = args.num
    print(f"start to extract images from time {args.start} and extract {extract_image_num} images")
    if extract_image_num < 0:
      print("Extract all images from ros bag")
      extract_image_num = total_num_imgs
    bridge = CvBridge()
    pbar = tqdm.tqdm(total=extract_image_num, colour="green")
    image_counter = 0
    extract_image_counter = 0
    t0 = None
    for topic, msg, t in bag.read_messages():
        if t0 is None:
          t0 = t
        if (t - t0).to_sec() < args.start:
          continue
        if topic == "/oak_ffc_4p/expose_time_us":
          if msg._type == "std_msgs/Int32":
            expose_time_ms = msg.data
        if topic == extract_image_topic:
            if(extract_image_counter >= extract_image_num):
               break
            image_counter+=1
            if(image_counter%args.step!=0):
                continue
            else:
              extract_image_counter+=1
            #   print("write topic",topic)
              if msg._type == "sensor_msgs/Image":
                img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
              if msg._type == "sensor_msgs/CompressedImage":
                img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')

              cv.imwrite(genereate_imagename(extract_image_counter,args.output_dir),img)
              file.write("{:05d}".format(extract_image_counter) + " "+ str(t.to_sec()) + " "+str(expose_time_ms) + "\n")
              pbar.update(1)
    pbar.close()
    file.close()
    print("zipping image folder please wait for exit\n")
    zip_folder(args.output_dir + "/images/",args.output_dir + "/images.zip")
    print("Finish extract images from bag file\n")
            # c += 1
            # if c % args.step != 0:
            #     continue
            # if msg._type == "sensor_msgs/Image":
            #     img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # else:
            #     img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # # imgs = split_image(img)
            # #Compress and write imgs to output bag
            # # comp_img = bridge.cv2_to_compressed_imgmsg(img)
            # comp_img = msg
            # comp_img.header = msg.header
            # outbag.write(topic, comp_img, t)
            # for i, _img in enumerate(imgs):
            #     if i == 3:
            #         comp_img = bridge.cv2_to_compressed_imgmsg(_img)
            #         comp_img.header = msg.header
            #         outbag.write(f"/arducam/image_{i}/compressed", comp_img, t)
                # cv.imwrite(f"/home/xuhao/output/quadvins-output/imgs/fisheye_{c:06d}_{i}.jpg", _img)
            # if args.show:
            #     for i in range(len(imgs)):
            #         cv.imshow(f"{topic}-{i}", imgs[i])
            #     cv.imshow(topic, img)
            #     cv.waitKey(1)
            # Update progress bar
            # pbar.update(1)