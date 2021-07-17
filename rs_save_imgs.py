# -*- coding: utf-8 -*-
# @Time    : 2021/7/17 上午10:39
# @Author  : yann
# @File    : rs_save_imgs.py
# @Project: rs_save_imgs


import pyrealsense2 as rs
import numpy as np
import cv2
import time

path = '/home/yann/Code/python/rs_save_imgs/img/'


def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
    config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

    pipeline.start(config)
    # 等待30帧
    for i in range(30):
        pipeline.wait_for_frames()

    while True:
        # 获取图像
        data = pipeline.wait_for_frames()
        depth = data.get_depth_frame()
        color = data.get_color_frame()
        ir_left = data.get_infrared_frame(1)
        ir_right = data.get_infrared_frame(2)

        # 获取内参
        dprofile = depth.get_profile()
        cprofile = color.get_profile()
        lprofile = ir_left.get_profile()
        rprofile = ir_right.get_profile()

        cvsprofile = rs.video_stream_profile(cprofile)
        dvsprofile = rs.video_stream_profile(dprofile)
        lvsprofile = rs.video_stream_profile(lprofile)
        rvsprofile = rs.video_stream_profile(rprofile)

        color_intrin = cvsprofile.get_intrinsics()
        print(color_intrin)
        depth_intrin = dvsprofile.get_intrinsics()
        print(depth_intrin)
        ir_left_intrin = lvsprofile.get_intrinsics()
        print(ir_left_intrin)
        ir_right_intrin = rvsprofile.get_intrinsics()
        print(ir_right_intrin)

        # 外参
        # extrin = dprofile.get_extrinsics_to(cprofile)
        # print(extrin)
        extrin = lprofile.get_extrinsics_to(rprofile)
        print(extrin)

        depth_image = np.asanyarray(depth.get_data())
        color_image = np.asanyarray(color.get_data())
        ir_left_image = np.asanyarray(ir_left.get_data())
        ir_right_image = np.asanyarray(ir_right.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Show images
        images1 = np.hstack((color_image, depth_colormap))
        images2 = np.hstack((ir_left_image, ir_right_image))
        cv2.imshow('images1', images1)
        cv2.imshow('images2', images2)

        key = cv2.waitKey(1)
        if key & 0xff == ord('q') or key == 27:
            break
        elif key & 0xff == ord('s'):
            # 时间戳，来区分文件
            t = time.time()
            tname = str(t)[0:13]

            cv2.imwrite(path + 'ir_left_image' + str(tname) + '.png', ir_left_image)
            cv2.imwrite(path + 'ir_right_image' + str(tname) + '.png', ir_right_image)
            cv2.imwrite(path + 'color' + str(tname) + '.png', color_image)
            cv2.imwrite(path + 'depth' + str(tname) + '.png', depth_image)
            cv2.imwrite(path + 'depth_colorMAP' + str(tname) + '.png', depth_colormap)


if __name__ == "__main__":
    main()



