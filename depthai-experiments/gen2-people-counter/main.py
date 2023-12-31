#!/usr/bin/env python3

from depthai_sdk import OakCamera, DetectionPacket, Visualizer, TextPosition
import cv2

with OakCamera(replay='people-images-01') as oak:
    color = oak.create_camera('color')
    nn = oak.create_nn('person-detection-retail-0013', color)
    oak.replay.setFps(3)

    def cb(packet: DetectionPacket, visualizer: Visualizer):
        num = len(packet.img_detections.detections)
        print('New msgs! Number of people detected:', num)

        visualizer.add_text(f"Number of people: {num}", position=TextPosition.TOP_MID)
        visualizer.draw(packet.frame)
        cv2.imshow(f'frame {packet.name}', packet.frame)

    oak.visualize(nn, callback=cb)
    # oak.show_graph()
    oak.start(blocking=True)
