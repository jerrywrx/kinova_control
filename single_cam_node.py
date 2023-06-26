import os
import sys
sys.path.append('/home/yixuan/dynamic_repr/NeuRay')
sys.path.append('/usr/lib/python3/dist-packages')

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import time
from sys import maxsize

import cv2
import depthai as dai
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import contextlib



import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--idx', type=int)

flags = parser.parse_args()

rospy.init_node('data_coll', anonymous=True)
br = CvBridge()

camera_params_2160_3840_list = [[3.13947962e+03, 3.13819822e+03, 1.93376762e+03, 1.10172115e+03], # fx, fy, cx, cy for cam 1
                                [3.13020348e+03, 3.13021909e+03, 1.92066929e+03, 1.08222144e+03], # fx, fy, cx, cy for cam 2
                                [3.11868848e+03, 3.11733411e+03, 1.91153069e+03, 1.05280843e+03], # fx, fy, cx, cy for cam 3
                                [3.13575743e+03, 3.13306467e+03, 1.96104439e+03, 1.08789233e+03]] # fx, fy, cx, cy for cam 4]
print('camera_params: ', camera_params_2160_3840_list)
print('WARNING: camera_params_2160_3840_list is hard coded, please check it before running the code')

COLOR = True

lrcheck = True  # Better handling for occlusions
extended = False  # Closer-in minimum depth, disparity range is doubled
subpixel = True  # Better accuracy for longer distance, fractional disparity 32-levels
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7
median = dai.StereoDepthProperties.MedianFilter.KERNEL_7x7

print("StereoDepth config options:")
print("    Left-Right check:  ", lrcheck)
print("    Extended disparity:", extended)
print("    Subpixel:          ", subpixel)
print("    Median filtering:  ", median)

def createPipeline(device):
    fps = 10
    pipeline = dai.Pipeline()

    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoLeft.setFps(fps)

    monoRight = pipeline.create(dai.node.MonoCamera)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    monoRight.setFps(fps)

    stereo = pipeline.createStereoDepth()
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.initialConfig.setMedianFilter(median)
    # stereo.initialConfig.setConfidenceThreshold(255)

    stereo.setLeftRightCheck(lrcheck)
    stereo.setExtendedDisparity(extended)
    stereo.setSubpixel(subpixel)
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    config = stereo.initialConfig.get()
    config.postProcessing.speckleFilter.enable = False
    config.postProcessing.speckleFilter.speckleRange = 50
    config.postProcessing.temporalFilter.enable = True
    config.postProcessing.spatialFilter.enable = True
    config.postProcessing.spatialFilter.holeFillingRadius = 2
    config.postProcessing.spatialFilter.numIterations = 1
    config.postProcessing.thresholdFilter.minRange = 400
    config.postProcessing.thresholdFilter.maxRange = 200000
    config.postProcessing.decimationFilter.decimationFactor = 1
    stereo.initialConfig.set(config)

    xout_depth = pipeline.createXLinkOut()
    xout_depth.setStreamName("depth")
    stereo.depth.link(xout_depth.input)

    # xout_disparity = pipeline.createXLinkOut()
    # xout_disparity.setStreamName('disparity')
    # stereo.disparity.link(xout_disparity.input)

    xout_colorize = pipeline.createXLinkOut()
    xout_colorize.setStreamName("colorize")
    xout_rect_left = pipeline.createXLinkOut()
    xout_rect_left.setStreamName("rectified_left")
    xout_rect_right = pipeline.createXLinkOut()
    xout_rect_right.setStreamName("rectified_right")

    if COLOR:
        camRgb = pipeline.create(dai.node.ColorCamera)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setIspScale(1, 3)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        camRgb.setFps(fps)
        camRgb.initialControl.setManualFocus(130)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        camRgb.isp.link(xout_colorize.input)
    else:
        stereo.rectifiedRight.link(xout_colorize.input)

    stereo.rectifiedLeft.link(xout_rect_left.input)
    stereo.rectifiedRight.link(xout_rect_right.input)
    return pipeline

class HostSync:
    def __init__(self):
        self.arrays = {}

    def add_msg(self, name, msg):
        if not name in self.arrays:
            self.arrays[name] = []
        # Add msg to array
        self.arrays[name].append({"msg": msg, "seq": msg.getSequenceNum()})

        synced = {}
        for name, arr in self.arrays.items():
            for i, obj in enumerate(arr):
                if msg.getSequenceNum() == obj["seq"]:
                    synced[name] = obj["msg"]
                    break
        # If there are 5 (all) synced msgs, remove all old msgs
        # and return synced msgs
        if len(synced) == 4:  # color, left, right, depth, nn
            # Remove old msgs
            for name, arr in self.arrays.items():
                for i, obj in enumerate(arr):
                    if obj["seq"] < msg.getSequenceNum():
                        arr.remove(obj)
                    else:
                        break
            return synced
        return False

w = 640
h = 360

with contextlib.ExitStack() as stack:
    deviceInfos = dai.Device.getAllAvailableDevices()
    print('Found ', len(deviceInfos), ' devices')
    usbSpeed = dai.UsbSpeed.SUPER
    openVinoVersion = dai.OpenVINO.Version.VERSION_2021_4
    
    dev_idx = flags.idx
    deviceInfo = deviceInfos[0]
    color_pub = rospy.Publisher(f'color_{dev_idx}', Image, queue_size=10)
    depth_pub = rospy.Publisher(f'depth_{dev_idx}', Image, queue_size=10)
        
    device: dai.Device = stack.enter_context(dai.Device(openVinoVersion, deviceInfo, usbSpeed))
    pipeline = createPipeline(device)
    device.startPipeline(pipeline)

    device.setIrLaserDotProjectorBrightness(1200)
        
    qs = []
    qs.append(device.getOutputQueue("depth", maxSize=1, blocking=False))
    qs.append(device.getOutputQueue("colorize", maxSize=1, blocking=False))
    qs.append(device.getOutputQueue("rectified_left", maxSize=1, blocking=False))
    qs.append(device.getOutputQueue("rectified_right", maxSize=1, blocking=False))

    # calibData = device.readCalibration()
    # if COLOR:
    #     w, h = camRgb.getIspSize()
    #     intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RGB, dai.Size2f(w, h))
    #     print(intrinsics)
    #     print(w, h)
    # else:
    #     w, h = monoRight.getResolutionSize()
    #     intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT, dai.Size2f(w, h))
    # intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.RGB, dai.Size2f(w, h))
    # camera_params = (intrinsics[0][0], intrinsics[1][1], intrinsics[0][2], intrinsics[1][2])
    camera_params = [camera_params_2160_3840_list[dev_idx][0] * w / 3840, camera_params_2160_3840_list[dev_idx][1] * h / 2160,
                        camera_params_2160_3840_list[dev_idx][2] * w / 3840, camera_params_2160_3840_list[dev_idx][3] * h / 2160]
    # pcl_converter = PointCloudVisualizer()

    serial_no = device.getMxId()
    sync = HostSync()
    depth_vis, color, rect_left, rect_right = None, None, None, None

    frame_idx = 0
    drop_start = 10
    while True:
        for q in qs:
            new_msg = q.tryGet()
            if new_msg is not None:
                msgs = sync.add_msg(q.getName(), new_msg)
                if msgs:
                    depth = msgs["depth"].getFrame() * 0.001
                    # plt.imshow(depth)
                    # plt.show()
                    color = msgs["colorize"].getCvFrame()
                    
                    # plt.subplot(1, 2, 1)
                    # plt.imshow(color)
                    # plt.subplot(1, 2, 2)
                    # plt.imshow(depth)
                    # plt.show()
                    
                    # rectified_left = msgs["rectified_left"].getCvFrame()
                    # rectified_right = msgs["rectified_right"].getCvFrame()
                    # depth_vis = cv2.normalize(depth, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
                    # depth_vis = cv2.equalizeHist(depth_vis)
                    # depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_HOT)
                    # cv2.imshow(f"depth_{device_idx}", depth_vis)
                    # cv2.imshow(f"color_{device_idx}", color)
                    
                    # key = cv2.waitKey(1)
                    # if key == ord("q"):
                    #     exit(0)
                    
                    header = Header()
                    header.stamp = rospy.Time.now()
                    header.frame_id = f"camera_{dev_idx}"
                    color_msg = br.cv2_to_imgmsg(color)
                    color_msg.header = header
                    color_pub.publish(color_msg)
                    depth_msg = br.cv2_to_imgmsg((depth * 1000).astype(np.uint16))
                    depth_msg.header = header
                    depth_pub.publish(depth_msg)
