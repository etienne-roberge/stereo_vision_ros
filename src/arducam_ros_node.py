#!/usr/bin/env python
import os
import time
import cv2
import numpy as np
import json
from ImageConvert import *
import arducam_config_parser
import ArducamSDK
import copy

import threading
import sys

import rospy
from cv_bridge import CvBridge, CvBridgeError
from camera_info_manager import CameraInfoManager
from sensor_msgs.msg import CameraInfo, Image

from arducam_usb2_ros.srv import WriteReg, WriteRegResponse
from arducam_usb2_ros.srv import ReadReg, ReadRegResponse
from arducam_usb2_ros.srv import Capture, CaptureResponse


global cfg,handle,Width,Heigth,color_mode, running
running = True
cfg = {}
handle = {}


def configBoard(config):
    global handle
    ArducamSDK.Py_ArduCam_setboardConfig(handle, config.params[0], \
        config.params[1], config.params[2], config.params[3], \
            config.params[4:config.params_length])

pass

def camera_initFromFile(fileName):
    global cfg,handle,Width,Height,color_mode
    #load config file
    config = arducam_config_parser.LoadConfigFile(fileName)

    camera_parameter = config.camera_param.getdict()
    Width = camera_parameter["WIDTH"]
    Height = camera_parameter["HEIGHT"]

    BitWidth = camera_parameter["BIT_WIDTH"]
    ByteLength = 1
    if BitWidth > 8 and BitWidth <= 16:
        ByteLength = 2
    FmtMode = camera_parameter["FORMAT"][0]
    color_mode = camera_parameter["FORMAT"][1]
    print("color mode",color_mode)

    I2CMode = camera_parameter["I2C_MODE"]
    I2cAddr = camera_parameter["I2C_ADDR"]
    TransLvl = camera_parameter["TRANS_LVL"]
    cfg = {"u32CameraType": 0x00,
            "u32Width":Width,"u32Height":Height,
            "usbType":0,
            "u8PixelBytes":ByteLength,
            "u16Vid":0,
            "u32Size":0,
            "u8PixelBits":BitWidth,
            "u32I2cAddr":I2cAddr,
            "emI2cMode":I2CMode,
            "emImageFmtMode":FmtMode,
            "u32TransLvl":TransLvl }

    # ArducamSDK.

    if serial_selection == None:
        ret,handle,rtn_cfg = ArducamSDK.Py_ArduCam_autoopen(cfg)
    else:
        index = None
        num_cam, cam_list, cam_serial = ArducamSDK.Py_ArduCam_scan()
        for i in range(num_cam):
            datas = cam_serial[i]
            camera_serial = "%c%c%c%c-%c%c%c%c-%c%c%c%c"%(datas[0],datas[1],datas[2],datas[3],
                                                          datas[4],datas[5],datas[6],datas[7],
                                                          datas[8],datas[9],datas[10],datas[11])
            if camera_serial == str(serial_selection):
                print("Arducam " + str(serial_selection) + " found")
                index = i
                break
        if index == None:
            print("Arducam " + str(serial_selection) + " not found")
            exit()
        time.sleep(3)
        ret,handle,rtn_cfg = ArducamSDK.Py_ArduCam_open(cfg,i)

    if ret == 0:
        usb_version = rtn_cfg['usbType']
        print("USB VERSION:" + str(usb_version))
        configs = config.configs
        configs_length = config.configs_length
        for i in range(configs_length):
            type = configs[i].type
            if ((type >> 16) & 0xFF) != 0 and ((type >> 16) & 0xFF) != usb_version:
                continue
            if type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_REG:
                ArducamSDK.Py_ArduCam_writeSensorReg(handle, configs[i].params[0], configs[i].params[1])
            elif type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_DELAY:
                time.sleep(float(configs[i].params[0])/1000)
            elif type & 0xFFFF == arducam_config_parser.CONFIG_TYPE_VRCMD:
                configBoard(configs[i])

        rtn_val,datas = ArducamSDK.Py_ArduCam_readUserData(handle,0x400-16, 16)
        print("Serial: %c%c%c%c-%c%c%c%c-%c%c%c%c"%(datas[0],datas[1],datas[2],datas[3],
                                                    datas[4],datas[5],datas[6],datas[7],
                                                    datas[8],datas[9],datas[10],datas[11]))

        return True
    else:
        print("open fail,rtn_val = ",ret)
        return False

pass


def captureImage_thread():
    global handle, running

    rtn_val = ArducamSDK.Py_ArduCam_beginCaptureImage(handle)
    if rtn_val != 0:
        print("Error beginning capture, rtn_val = ", rtn_val)
        running = False
        return
    else:
        print("Capture began, rtn_val = ", rtn_val)

    while running:
        # print "capture"
        rtn_val = ArducamSDK.Py_ArduCam_captureImage(handle)
        if rtn_val > 255:
            print("Error capture image, rtn_val = ", rtn_val)
            if rtn_val == ArducamSDK.USB_CAMERA_USB_TASK_ERROR:
                break

    running = False
    ArducamSDK.Py_ArduCam_endCaptureImage(handle)

def splitStereoImage(stereo_image):
    """
    Split a stereo pair
    :param stereo_image: a frame from utils_arducam.getFrame()
    :return: left and right original images
    """
    imgWidth = stereo_image.shape[1] // 2
    imgL = stereo_image[:, :imgWidth, :]
    imgR = stereo_image[:, imgWidth:, :]
    return imgL, imgR

def readImage_thread():
    global handle, running, Width, Height, cfg, color_mode, save_raw
    global COLOR_BayerGB2BGR, COLOR_BayerRG2BGR, COLOR_BayerGR2BGR, COLOR_BayerBG2BGR
    count = 0
    totalFrame = 0
    time0 = time.time()
    time1 = time.time()
    data = {}
    #cv2.namedWindow("ArduCam Demo", 1)
    if not os.path.exists("images"):
        os.makedirs("images")
    while running:
        display_time = time.time()
        if ArducamSDK.Py_ArduCam_availableImage(handle) > 0:
            rtn_val, data, rtn_cfg = ArducamSDK.Py_ArduCam_readImage(handle)
            datasize = rtn_cfg['u32Size']
            if rtn_val != 0 or datasize == 0:
                print("read data fail!")
                ArducamSDK.Py_ArduCam_del(handle)
                return

            image = convert_image(data, rtn_cfg, color_mode)

            if h_flip:
                image = cv2.flip(image, 1)
            if v_flip:
                image = cv2.flip(image, 0)

            time1 = time.time()
            if time1 - time0 >= 1:
                print("%s %d %s\n" % ("fps:", count, "/s"))
                count = 0
                time0 = time1
            count += 1


            left, right = splitStereoImage(image)
            imgL = copy.copy(left)
            imgR = copy.copy(right)

            try:
                img_msg = bridge.cv2_to_imgmsg(imgL, "bgr8")
                #img_msg = bridge.cv2_to_imgmsg(imgL)
                img_msg.header.stamp = rospy.Time.now()
                img_msg.header.frame_id = id_frame
                pubL.publish(img_msg)
                img_msg = bridge.cv2_to_imgmsg(imgR, "bgr8")
                #img_msg = bridge.cv2_to_imgmsg(imgR)
                img_msg.header.stamp = rospy.Time.now()
                img_msg.header.frame_id = id_frame
                pubR.publish(img_msg)
            except CvBridgeError as e:
                print(e)
                pass

            #imgL = cv2.resize(imgL, (640, 480), interpolation=cv2.INTER_LINEAR)
            #cv2.imshow("ArduCam Demo", imgL)

            cv2.waitKey(10)
            ArducamSDK.Py_ArduCam_del(handle)
        else:
            time.sleep(0.001)

def sigint_handler(signum, frame):
    global running,handle
    running = False
    exit()

def write_register(request):
    global handle
    register = request.register
    value = request.value
    rtn_val = ArducamSDK.Py_ArduCam_writeSensorReg(handle,register,value)
    if rtn_val == 0:
        output = 'Value %d written to register %d' % (value, register)
    else:
        output = 'Invalid register'
    return WriteRegResponse(output)

def read_register(request):
    global handle
    register = request.register
    rtn_val, output = ArducamSDK.Py_ArduCam_readSensorReg(handle,register)
    if rtn_val == 0:
        output = 'Register %d: %d' % (register, output)
    else:
        output = 'Invalid register'
    return ReadRegResponse(output)

def rosShutdown():
    global handle
    ArducamSDK.Py_ArduCam_endCaptureImage(handle)
    rtn_val = ArducamSDK.Py_ArduCam_close(handle)
    if rtn_val == 0:
        print("device close success!")
    else:
        print("device close fail!")

if __name__ == "__main__":

    rospy.init_node("stereo_camera_ros_node")
    pubL = rospy.Publisher("left/image_raw", Image, queue_size=1)
    pubR = rospy.Publisher("right/image_raw", Image, queue_size=1)
    left_ci = CameraInfoManager(cname='left', url='file://tmp', namespace="left")
    left_ci.loadCameraInfo()
    camInfo = left_ci.getCameraInfo()
    pubL_info = rospy.Publisher("left/camera_info", CameraInfo, queue_size=1)
    right_ci = CameraInfoManager(cname='right_camera', namespace='right')
    bridge = CvBridge()
    try:
        config_file_name = rospy.get_param("~config_file")
        if not os.path.exists(config_file_name):
            print("Config file does not exist.")
            exit()
    except:
        print("Empty config_file parameter.")
        exit()
    try:
        serial_selection = rospy.get_param("~camera_serial")
        if len(serial_selection) == 0:
            raise
    except:
        serial_selection = None
    try:
        h_flip = rospy.get_param("~horizontal_flip")
    except:
        h_flip = False
    try:
        v_flip = rospy.get_param("~vertical_flip")
    except:
        v_flip = False
    try:
        id_frame = rospy.get_param("~frame_id")
    except:
        print("Please input frame_id.")
        exit()

    if camera_initFromFile(config_file_name):
        ArducamSDK.Py_ArduCam_setMode(handle,ArducamSDK.CONTINUOUS_MODE)

        ct = threading.Thread(target=captureImage_thread)
        rt = threading.Thread(target=readImage_thread)
        ct.start()
        rt.start()

        rtn_val = ArducamSDK.Py_ArduCam_beginCaptureImage(handle)
        if rtn_val != 0:
            print("Error beginning capture, rtn_val = ",rtn_val)
            exit()
        else:
            print("Capture began, rtn_val = ",rtn_val)

        service_write = rospy.Service('write_reg', WriteReg, write_register)
        service_read = rospy.Service('read_reg', ReadReg, read_register)

        rospy.on_shutdown(rosShutdown)

        while running:
            camInfo = left_ci.getCameraInfo()
            pubL_info.publish(camInfo)
            time.sleep(1) #spin dans le beurre

        ct.join()
        rt.join()

        if rtn_val == 0:
            print("device close success!")
        else:
            print("device close fail!")
