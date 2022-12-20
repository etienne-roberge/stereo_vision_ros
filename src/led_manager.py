

class LEDManager:

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

            ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3503, 0b00000111)

            ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3500, 0b11111111)
            ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3501, 0b11111111)
            ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3502, 0b01111111)

            # Darker
            ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x350A, 0b00000000)
            ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x350B, 0b00000000)
            ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3500, 0b00000000)
            ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3501, 0b00000111)
            #ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x3502, 0b00000000)

            # Brighter
            #ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x350A, 0b00011000)
            #ArducamSDK.Py_ArduCam_writeSensorReg(self.handle, 0x350B, 0b00111100)
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