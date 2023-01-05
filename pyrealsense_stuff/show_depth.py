import logging
logging.basicConfig(level=logging.INFO)

import time
import numpy as np
import cv2
import pyrealsense as pyrs
from pyrealsense.constants import rs_option

depth_fps = 90
depth_stream = pyrs.stream.DepthStream(fps=depth_fps)
index = 0

def convert_z16_to_bgr(frame):
    '''Performs depth histogram normalization

    This raw Python implementation is slow. See here for a fast implementation using Cython:
    https://github.com/pupil-labs/pupil/blob/master/pupil_src/shared_modules/cython_methods/methods.pyx
    '''
    hist = np.histogram(frame, bins=0x10000)[0]
    hist = np.cumsum(hist)
    hist -= hist[0]
    rgb_frame = np.empty(frame.shape[:2] + (3,), dtype=np.uint8)

    zeros = frame == 0
    non_zeros = frame != 0

    f = hist[frame[non_zeros]] * 255 / hist[0xFFFF]
    rgb_frame[non_zeros, 0] = 255 - f
    rgb_frame[non_zeros, 1] = 0
    rgb_frame[non_zeros, 2] = f
    rgb_frame[zeros, 0] = 20
    rgb_frame[zeros, 1] = 5
    rgb_frame[zeros, 2] = 0
    return rgb_frame


with pyrs.Service() as serv:
    with serv.Device() as dev:

        dev.apply_ivcam_preset(0)

        try:  # set custom gain/exposure values to obtain good depth image
            custom_options = [(rs_option.RS_OPTION_R200_LR_EXPOSURE, 501.0),
                              (rs_option.RS_OPTION_R200_LR_GAIN, 330.0)] #ADICIONAR A VERSÃO LONGE RANGE COMO PARAMETROS!
            dev.set_device_options(*zip(*custom_options))
        except pyrs.RealsenseError:
            pass  # options are not available on all devices

        cnt = 0
        last = time.time()
        smoothing = 0.9
        fps_smooth = depth_fps

        while True:

            cnt += 1
            index += 1
            if (cnt % 30) == 0:
                now = time.time()
                dt = now - last
                fps = 30/dt
                fps_smooth = (fps_smooth * smoothing) + (fps * (1.0-smoothing))
                last = now

            dev.wait_for_frames()

            d = dev.depth
            
            d = convert_z16_to_bgr(d)
            #d = dev.depth * dev.depth_scale * 1000
            
            
            color = dev.color
            color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
            
            if index % 20 == 0:
                cv2.imwrite('C:/Users/jpedr/Documents/Python/RGBD_realsense-experiments/depth_color/depth'+str(index)+'.jpg', d)
                cv2.imwrite('C:/Users/jpedr/Documents/Python/RGBD_realsense-experiments/color'+str(index)+'.jpg', color)
            
            cv2.putText(d, str(fps_smooth)[:4], (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255))

            cv2.imshow('depth', d)
            cv2.imshow('color', color)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            