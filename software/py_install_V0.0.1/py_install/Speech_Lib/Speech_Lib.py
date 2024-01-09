#!/usr/bin/env python3
# coding: utf-8

import time
import threading
import sys
import serial


# V0.0.1
class Speech(object):

    def __init__(self, com="/dev/myspeech"):
        # com="/dev/ttyUSB0"
        self.ser = serial.Serial(com, 115200)
        if self.ser.isOpen():
            print("Speech Serial Opened! Baudrate=115200")
        else:
            print("Speech Serial Open Failed!")

    def __del__(self):
        self.ser.close()
        print("speech serial Close!")

    # 选择播报语句
    def void_write(self, void_data):
        void_data1 = int(void_data/100)+48
        void_data2 = int(void_data%100/10)+48
        void_data3 = int(void_data%10)+48
        cmd = [0x24, 0x41, void_data1, void_data2, void_data3, 0x23]
        #print(cmd)
        self.ser.write(cmd)
        time.sleep(0.005)
        self.ser.flushInput()

    # 读取识别的语音
    def speech_read(self):
        count = self.ser.inWaiting()
        if count:
            speech_data = self.ser.read(count)
            speech_data1 = int(str(speech_data)[4:5])
            speech_data2 = int(str(speech_data)[5:6])
            speech_data3 = int(str(speech_data)[6:7])
            self.ser.flushInput()
            time.sleep(0.005)
            print(int(speech_data1*100+speech_data2*10+speech_data3))	
            return int(speech_data1*100+speech_data2*10+speech_data3) 
        else:
            return 999
