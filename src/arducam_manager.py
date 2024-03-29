#!/usr/bin/env python
import time
from ImageConvert import *
import arducam_config_parser
import ArducamSDK
import copy
import threading
import rospy

class ArducamManager:

    def __init__(self, camera_config):
        # load config file
        self.running = False
        self.captureThread = None
        config = arducam_config_parser.LoadConfigFile(camera_config)
        camera_parameter = config.camera_param.getdict()

        self.width = camera_parameter["WIDTH"]
        self.height = camera_parameter["HEIGHT"]

        BitWidth = camera_parameter["BIT_WIDTH"]
        ByteLength = 1
        if 8 < BitWidth <= 16:
            ByteLength = 2
        FmtMode = camera_parameter["FORMAT"][0]
        self.color_mode = camera_parameter["FORMAT"][1]
        rospy.loginfo("color mode" + str(self.color_mode))

        I2CMode = camera_parameter["I2C_MODE"]
        I2cAddr = camera_parameter["I2C_ADDR"]
        TransLvl = camera_parameter["TRANS_LVL"]
        self.cfg = {"u32CameraType": 0x00,
                    "u32Width": self.width, "u32Height": self.height,
                    "usbType": 0,
                    "u8PixelBytes": ByteLength,
                    "u16Vid": 0,
                    "u32Size": 0,
                    "u8PixelBits": BitWidth,
                    "u32I2cAddr": I2cAddr,
                    "emI2cMode": I2CMode,
                    "emImageFmtMode": FmtMode,
                    "u32TransLvl": TransLvl}

        ret, self.handle, rtn_cfg = ArducamSDK.Py_ArduCam_autoopen(self.cfg)

        if ret == 0:
            usb_version = rtn_cfg['usbType']
            rospy.loginfo("USB VERSION:" + str(usb_version))
            configs = config.configs
            configs_length = config.configs_length
            for i in range(configs_length):
                configType = configs[i].type
                if ((configType >> 16) & 0xFF) != 0 and ((configType >> 16) & 0xFF) != usb_version:
                    continue
                if configType & 0xFFFF == arducam_config_parser.CONFIG_TYPE_REG:
                    ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, configs[i].params[0], configs[i].params[1])
                elif configType & 0xFFFF == arducam_config_parser.CONFIG_TYPE_DELAY:
                    time.sleep(float(configs[i].params[0]) / 1000)
                elif configType & 0xFFFF == arducam_config_parser.CONFIG_TYPE_VRCMD:
                    self.configBoard(configs[i])

            rtn_val, datas = ArducamSDK.Py_ArduCam_readUserData(self.handle, 0x400 - 16, 16)
            rospy.loginfo("Serial: %c%c%c%c-%c%c%c%c-%c%c%c%c" % (datas[0], datas[1], datas[2], datas[3],
                                                          datas[4], datas[5], datas[6], datas[7],
                                                          datas[8], datas[9], datas[10], datas[11]))

            ArducamSDK.Py_ArduCam_registerCtrls(self.handle, config.controls, config.controls_length)

            ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3503, 0b00000000)
            # ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3503, 0b00000111)
            # ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x350A, 0b00000000)
            # ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x350B, 0b00000000)
            #
            # ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3500, 0b11111111)
            # ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3501, 0b11111111)
            # ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3502, 0b01111111)
        else:
            rospy.logfatal("open fail,rtn_val = " + str(ret))
            exit(1)

    def configBoard(self, config):
        ArducamSDK.Py_ArduCam_setboardConfig(self.handle, config.params[0],
                                             config.params[1], config.params[2], config.params[3],
                                             config.params[4:config.params_length])

    def startCapture(self):
        ArducamSDK.Py_ArduCam_setMode(self.handle, ArducamSDK.CONTINUOUS_MODE)
        self.running = True
        self.captureThread = threading.Thread(target=self.captureImageThread)
        self.captureThread.start()

    def shutdown(self):
        self.running = False
        self.captureThread.join()
        rtn_val = ArducamSDK.Py_ArduCam_close(self.handle)
        if rtn_val == 0:
            rospy.loginfo("device close success!")
        else:
            rospy.logfatal("device close fail!")

    def captureImageThread(self):
        rtn_val = ArducamSDK.Py_ArduCam_beginCaptureImage(self.handle)
        if rtn_val != 0:
            rospy.logfatal("Error beginning capture, rtn_val = " + str(rtn_val))
            self.running = False
            return
        else:
            rospy.loginfo("Capture began, rtn_val = " + str(rtn_val))

        while self.running:
            rtn_val = ArducamSDK.Py_ArduCam_captureImage(self.handle)
            if rtn_val > 255:
                rospy.logwarn("Error capture image, rtn_val = " + str(rtn_val))
                if rtn_val == ArducamSDK.USB_CAMERA_USB_TASK_ERROR:
                    rospy.logfatal("USB_CAMERA_USB_TASK_ERROR!")
                    break

        self.running = False
        ArducamSDK.Py_ArduCam_endCaptureImage(self.handle)

    def readImages(self):
        if ArducamSDK.Py_ArduCam_availableImage(self.handle) > 0:
            rtn_val, data, rtn_cfg = ArducamSDK.Py_ArduCam_readImage(self.handle)
            datasize = rtn_cfg['u32Size']
            if rtn_val != 0 or datasize == 0:
                rospy.logfatal("read data fail!")
                ArducamSDK.Py_ArduCam_del(self.handle)
                exit(1)

            image = convert_image(data, rtn_cfg, self.color_mode)
            image = cv2.resize(image, (1440, 480))

            # split stereo images
            imgWidth = image.shape[1] // 2
            imgHeight = image.shape[0]
            left = image[:imgHeight, :imgWidth, :]
            right = image[:imgHeight, imgWidth:, :]
            #left = image[:imgHeight, 100:imgWidth, :]
            #right = image[:imgHeight, imgWidth:image.shape[1]-100z, :]
            imgL = copy.copy(left)
            imgR = copy.copy(right)

            ArducamSDK.Py_ArduCam_flush(self.handle)
            return imgL, imgR
        else:
            return None, None

    def startTouch(self):
        ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3503, 0b00000111)
        ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x350A, 0b00000000)
        ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x350B, 0b00000000)
        ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3500, 0b00000000)
        ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3501, 0b00001000)
        ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3502, 0b01111111)

    def startVision(self):
        #ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3503, 0b00000000)

        ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3503, 0b00000111)
        ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x350A, 0b00000000)
        ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x350B, 0b00000000)

        ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3500, 0b11111111)
        ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3501, 0b11111111)
        ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3502, 0b01111111)