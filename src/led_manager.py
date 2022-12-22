import time

import serial
import threading

class LEDManager:
    def __init__(self):
        # load config file
        self.running = False
        self.arduinoThread = None
        self.startTouchFlag = False
        self.startVisionFlag = False

        self.serPort = serial.Serial("/dev/ttyUSB0")
        self.startArduinoThread()


    def startArduinoThread(self):
        self.running = True
        self.arduinoThread = threading.Thread(target=self.arduinoLoop)
        self.arduinoThread.start()

    def arduinoLoop(self):
        while self.running:
            if self.startTouchFlag:
                self.serPort.write("startTouch\n".encode())
                self.startTouchFlag = False
            elif self.startVisionFlag:
                self.serPort.write("startVision\n".encode())
                self.startVisionFlag = False
            else:
                time.sleep(0.1)
        self.running = False

    def shutdown(self):
        self.running = False
        self.arduinoThread.join()
        self.serPort.close()

    def startTouch(self):
        self.startTouchFlag = True
    def startVision(self):
        self.startVisionFlag = True