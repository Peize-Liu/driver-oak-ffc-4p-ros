#!/usr/bin/env python3
#to extract image with sepecific step from bag file. small the bag size
#usage: python3 quadcam_image_extract.py -i input.bag -s 10 -t 0.5
#only for oak_ffc_4p


import rosbag
from os.path import exists
from cv_bridge import CvBridge
import cv2 as cv
import tqdm
import numpy as np



def generate_bagname(bag, comp=False):
    from pathlib import Path
    p = Path(bag)
    bagname = p.stem + "-split.bag"
    output_bag = p.parents[0].joinpath(bagname)
    return output_bag

# def split_image(img, num_subimages = 4):
#     #Split image vertically
#     h, w = img.shape[:2]
#     sub_w = w // num_subimages
#     sub_imgs = []
#     for i in range(num_subimages):
#         sub_imgs.append(img[:, i*sub_w:(i+1)*sub_w])
#     return sub_imgs

topic_list = [
    "/oak_ffc_4p/image_CAM_A",
    "/oak_ffc_4p/image_CAM_B",
    "/oak_ffc_4p/image_CAM_C",
    "/oak_ffc_4p/image_CAM_D",
    "/oak_ffc_4p/image_CAM_A/compressed",
    "/oak_ffc_4p/image_CAM_B/compressed",
    "/oak_ffc_4p/image_CAM_C/compressed",
    "/oak_ffc_4p/image_CAM_D/compressed"]

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
    parser.add_argument("-i","--input", type=str, help="input bag file")
    parser.add_argument('-v', '--show', action='store_true', help='compress the image topics')
    parser.add_argument('-s', '--step', type=int, nargs="?", help="step for images, default 1", default=1)
    parser.add_argument('-t', '--start', type=float, nargs="?", help="start time of the first image, default 0", default=0)
    args = parser.parse_args()
    output_bag = generate_bagname(args.input)
    if not exists(args.input):
        print(f"Input bag file {args.input} does not exist")
        exit(1)
    
    
    bag = rosbag.Bag(args.input)
    total_num_imgs = 0
    for i in range(len(topic_list)):
      num_imgs = bag.get_message_count(topic_list[i])
      total_num_imgs += num_imgs
      print("%s has number of images:%d",topic_list[i] ,num_imgs)
    print("total number of images:%d",total_num_imgs)
    bridge = CvBridge()
    pbar = tqdm.tqdm(total=total_num_imgs/args.step, colour="green")
    with rosbag.Bag(output_bag, 'w') as outbag:
        from nav_msgs.msg import Path
        path = Path()
        path_arr = []
        c = 0
        t0 = None
        for topic, msg, t in bag.read_messages():
            if t0 is None:
                t0 = t
            if (t - t0).to_sec() < args.start:
                continue
            
            if topic in topic_list:
                topic_conter[topic]+=1
                if(topic_conter[topic]%args.step!=0):
                    continue
                else:
                #   print("write topic",topic)
                  if msg._type == "sensor_msgs/Image":
                    img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                  if msg._type == "sensor_msgs/CompressedImage":
                    img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
                  img_msg = bridge.cv2_to_compressed_imgmsg(img)
                  comp_img = img_msg
                  comp_img.header = msg.header
                  outbag.write(topic, comp_img, t)
                  pbar.update(1)
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
            else:
                outbag.write(topic, msg, t)
                


