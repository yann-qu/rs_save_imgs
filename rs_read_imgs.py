# -*- coding: utf-8 -*-
# @Time    : 2021/7/17 上午10:39
# @Author  : yann
# @File    : rs_read_imgs.py
# @Project: rs_save_imgs

import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

pipeline.start(config)

for i in range(30):
    pipeline.wait_for_frames()

try:
    while True:
        frames = pipeline.wait_for_frames()           # type: rs.composite_frame
        depth_frame = frames.get_depth_frame()        # type: rs.frame
        color_frame = frames.get_color_frame()        # type: rs.frame
        ir_frame_left = frames.get_infrared_frame(1)  # type: rs.frame
        ir_frame_right = frames.get_infrared_frame(2) # type: rs.frame

        if not depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        ir_left_image = np.asanyarray(ir_frame_left.get_data())
        ir_right_image = np.asanyarray(ir_frame_right.get_data())

        # 渲染深度图
        # convertScaleAbs是把数组中每一项 ixa+b，然后把结果转换成8bit，变换后数值大于255的就限制在255
        # 255/0.03=8500，深度图渲染后大致可以看8m，8m外的颜色都一样了，分辨不出距离。
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.255), cv2.COLORMAP_JET)

        # 拼接图片
        images1 = np.hstack((color_image, depth_colormap))
        images2 = np.hstack((ir_left_image, ir_right_image))

        cv2.imshow('color and depth_color', images1)
        cv2.imshow('ir images', images2)

        key = cv2.waitKey(1)

        if key & 0xFF == ord('q') or key == 27:
            break

finally:
    pipeline.stop()



