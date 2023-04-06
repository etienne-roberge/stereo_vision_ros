import time

import serial
import threading
import rospy
import os
from std_msgs.msg import String

class LEDManager:
    def __init__(self):
        # load config file
        self.running = False
        self.serviceThread = None
        self.arduinoThread = None
        self.startTouchFlag = False
        self.startVisionFlag = False

        self.pub = rospy.Publisher('/stereo_vision/lights', String, queue_size=10)

        self.serPort = None

        try:
            for file in os.listdir("/dev/"):
                if file.startswith("ttyACM"):
                    self.serPort = serial.Serial("/dev/" + file, 9600, timeout=1)
                    self.startArduinoThread()
                    break
        except:
            rospy.logfatal("No LED controller found!")
            self.running = False


    def startArduinoThread(self):
        self.running = True
        self.serviceThread = threading.Thread(target=self.serviceLoop)
        self.arduinoThread = threading.Thread(target=self.arduinoLoop)
        self.arduinoThread.start()
        self.serviceThread.start()

    def arduinoLoop(self):
        while self.running:
            data = self.serPort.readline().decode('utf-8').rstrip()
            if len(data)>0:
                self.pub.publish(data)
        self.running = False
    def serviceLoop(self):
        while self.running:
            if self.startTouchFlag:
                self.serPort.write("t\n".encode())
                self.startTouchFlag = False
            elif self.startVisionFlag:
                self.serPort.write("s\n".encode())
                self.startVisionFlag = False
            else:
                time.sleep(0.1)
        self.running = False

    def shutdown(self):
        if self.running:
            self.running = False
            self.arduinoThread.join()
            self.serPort.write("s\n".encode())
            self.serPort.close()

    def startTouch(self):
        self.startTouchFlag = True
    def startVision(self):
        self.startVisionFlag = True