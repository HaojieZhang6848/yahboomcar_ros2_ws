#!/usr/bin/env python3
# coding=utf-8
import os
import struct
import sys
from Rosmaster_Lib import Rosmaster
import time
import threading

# V1.1.0-1
class Rosmaster_Joystick(object):

    def __init__(self, robot: Rosmaster, js_id=0, debug=False):
        self.__debug = debug
        self.__js_id = int(js_id)
        self.__js_isOpen = False
        self.__robot = robot
        self.__car_type = 1
        self.__car_type = self.__robot.CARTYPE_R2

        self.STATE_OK = 0
        self.STATE_NO_OPEN = 1
        self.STATE_DISCONNECT = 2
        self.STATE_KEY_BREAK = 3

        self.__IGNORE_MAX_COUNT = 24
        self.__ignore_count = self.__IGNORE_MAX_COUNT

        self.__strip_effect = 0
        self.__speed_x = 0
        self.__speed_y = 0
        self.__speed_z = 0
        self.__speed_ctrl_xy = 1
        self.__speed_ctrl_z = 1


        self.__MAX_SPEED = {
            "X3_X": 1.0,
            "X3_Y": 1.0,
            "X3_Z": 5.0,
            "X3PLUS_X": 0.7,
            "X3PLUS_Y": 0.7,
            "X3PLUS_Z": 3.2,
            "R2_X": 1.8,
            "R2_Y": 0.045,
            "R2_Z": 3.0,
        }
        

        # Find the joystick device.
        print('Joystick Available devices:')
        # Shows the joystick list of the Controler, for example: /dev/input/js0
        for fn in os.listdir('/dev/input'):
            if fn.startswith('js'):
                print('    /dev/input/%s' % (fn))

        # Open the joystick device.
        try:
            js = '/dev/input/js' + str(self.__js_id)
            self.__jsdev = open(js, 'rb')
            self.__js_isOpen = True
            print('---Opening %s Succeeded---' % js)
        except:
            self.__js_isOpen = False
            print('---Failed To Open %s---' % js)


        # Defining Functional List
        # self.__function_names = {
        #     # BUTTON FUNCTION
        #     0x0100 : 'A',
        #     0x0101 : 'B',
        #     0x0102 : 'X',
        #     0x0103 : 'Y',
        #     0x0104 : 'L1',
        #     0x0105 : 'R1',
        #     0x0106 : 'SELECT',
        #     0x0107 : 'START',
        #     0x0108 : 'MODE',
        #     0x0109 : 'BTN_RK1',
        #     0x010A : 'BTN_RK2',

        #     # AXIS FUNCTION
        #     0x0200 : 'RK1_LEFT_RIGHT',
        #     0x0201 : 'RK1_UP_DOWN',
        #     0x0202 : 'L2',
        #     0x0203 : 'RK2_LEFT_RIGHT',
        #     0x0204 : 'RK2_UP_DOWN',
        #     0x0205 : 'R2',
        #     0x0206 : 'WSAD_LEFT_RIGHT',
        #     0x0207 : 'WSAD_UP_DOWN',
        # }

        self.__function_names = {
            # BUTTON FUNCTION
            0x0100: 'A',
            0x0101: 'B',
            0x0103: 'X',
            0x0104: 'Y',
            0x0106: 'L1',
            0x0107: 'R1',
            0x0108: 'L2_1',
            0x0109: 'R2_1',
            0x010A: 'SELECT',
            0x010B: 'START',
            0x010D: 'BTN_RK1',                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
            0x010E: 'BTN_RK2',

            # AXIS FUNCTION
            0x0200: 'RK1_LEFT_RIGHT',
            0x0201: 'RK1_UP_DOWN',
            0x0202: 'RK2_LEFT_RIGHT',
            0x0203: 'RK2_UP_DOWN',
            0x0204: 'R2',
            0x0205: 'L2',
            0x0206: 'WSAD_LEFT_RIGHT',
            0x0207: 'WSAD_UP_DOWN',
        }

    def __del__(self):
        if self.__js_isOpen:
            self.__jsdev.close()
        if self.__debug:
            print("\n---Joystick DEL---\n")

    # Return joystick state
    def is_Opened(self):
        return self.__js_isOpen

    # reset
    def __robot_reset(self):
        self.__strip_effect = 0
        self.__speed_ctrl_xy = 1
        self.__speed_x = 0
        self.__speed_y = 0
        self.__speed_z = 0
        self.__robot.set_colorful_effect(self.__strip_effect, 5)
        self.__robot.set_car_run(7, 0)
        pass

    def my_map(self, x, in_min, in_max, out_min, out_max):
        return (out_max - out_min) * (x - in_min) / (in_max - in_min) + out_min

    # Control robot
    def __data_processing(self, name, value):
        if name == "RK1_LEFT_RIGHT":
            value = -value / 32767
            if self.__debug:
                print("%s : %.3f" % (name, value))
            if self.__car_type == self.__robot.CARTYPE_R2:
                self.__speed_z = value * self.__MAX_SPEED["R2_Z"]
            elif self.__car_type == self.__robot.CARTYPE_X3:
                self.__speed_y = value * self.__MAX_SPEED["X3_Y"] * self.__speed_ctrl_xy
            elif self.__car_type == self.__robot.CARTYPE_X3_PLUS:
                self.__speed_y = value * self.__MAX_SPEED["X3PLUS_Y"] * self.__speed_ctrl_xy
            else:
                return
            self.__robot.set_car_motion(self.__speed_x, self.__speed_y, self.__speed_z)

        elif name == 'RK1_UP_DOWN':
            value = -value / 32767
            if self.__debug:
                print("%s : %.3f" % (name, value))
            if self.__car_type == self.__robot.CARTYPE_R2:
                self.__speed_x = value * self.__MAX_SPEED["R2_X"] * self.__speed_ctrl_xy
            elif self.__car_type == self.__robot.CARTYPE_X3:
                self.__speed_x = value * self.__MAX_SPEED["X3_Y"] * self.__speed_ctrl_xy
            elif self.__car_type == self.__robot.CARTYPE_X3_PLUS:
                self.__speed_x = value * self.__MAX_SPEED["X3PLUS_Y"] * self.__speed_ctrl_xy
            else:
                return
            self.__robot.set_car_motion(self.__speed_x, self.__speed_y, self.__speed_z)

        elif name == 'RK2_LEFT_RIGHT':
            value = -value / 32767
            if self.__debug:
                print("%s : %.3f" % (name, value))
            if self.__car_type == self.__robot.CARTYPE_R2:
                self.__speed_y = value * self.__MAX_SPEED["R2_Y"]
                self.__speed_z = 0
            elif self.__car_type == self.__robot.CARTYPE_X3:
                self.__speed_z = value * self.__MAX_SPEED["X3_Z"] * self.__speed_ctrl_z
            elif self.__car_type == self.__robot.CARTYPE_X3_PLUS:
                self.__speed_z = value * self.__MAX_SPEED["X3PLUS_Z"] * self.__speed_ctrl_z
            else:
                return
            self.__robot.set_car_motion(self.__speed_x, self.__speed_y, self.__speed_z)

        elif name == 'RK2_UP_DOWN':
            value = -value / 32767
            if self.__debug:
                print("%s : %.3f" % (name, value))

        elif name == 'A':
            if self.__debug:
                print(name, ":", value)

        elif name == 'B':
            if self.__debug:
                print(name, ":", value)

        elif name == 'X':
            if self.__debug:
                print(name, ":", value)

        elif name == 'Y':
            if self.__debug:
                print(name, ":", value)

        elif name == 'L1':
            if self.__debug:
                print(name, ":", value)

        elif name == 'R1':
            if self.__debug:
                print(name, ":", value)
            if value == 1:
                self.__strip_effect = self.__strip_effect + 1
                if self.__strip_effect > 6:
                    self.__strip_effect = 0
                self.__robot.set_colorful_effect(self.__strip_effect, 5)

        elif name == 'SELECT':
            if self.__debug:
                print(name, ":", value)

        elif name == 'START':
            if self.__debug:
                print(name, ":", value)
            self.__robot.set_beep(value)

        elif name == 'MODE':
            if self.__debug:
                print(name, ":", value)

        elif name == 'BTN_RK1':
            if self.__debug:
                print(name, ":", value)
            if value == 1:
                self.__speed_ctrl_xy = self.__speed_ctrl_xy + 0.3
                if self.__speed_ctrl_xy > 1:
                    self.__speed_ctrl_xy = 0.3

        elif name == 'BTN_RK2':
            if self.__debug:
                print(name, ":", value)

        elif name == "L2":
            value = ((value/32767)+1)/2
            if self.__debug:
                print("%s : %.3f" % (name, value))

        elif name == "R2":
            value = ((value/32767)+1)/2
            if self.__debug:
                print("%s : %.3f" % (name, value))
            if value == 1:
                self.__robot_reset()

        elif name == 'WSAD_LEFT_RIGHT':
            value = -value / 32767
            if self.__debug:
                print("%s : %.3f" % (name, value))

        elif name == 'WSAD_UP_DOWN':
            value = -value / 32767
            if self.__debug:
                print("%s : %.3f" % (name, value))

        else:
            pass

    # Handles events for joystick
    def joystick_handle(self):
        if not self.__js_isOpen:
            # if self.__debug:
            #     print('Failed To Open Joystick')
            return self.STATE_NO_OPEN
        try:
            evbuf = self.__jsdev.read(8)
            if evbuf:
                timestamp, value, type, number = struct.unpack('IhBB', evbuf)
                func = type << 8 | number
                name = self.__function_names.get(func)
                print("evbuf:", timestamp, value, type, number)
                # if self.__debug:
                #     print("func:0x%04X, %s, %d" % (func, name, value))
                if name != None:
                    self.__data_processing(name, value)
                else:
                    if self.__ignore_count > 0:
                        self.__ignore_count = self.__ignore_count - 1
                    if self.__debug and self.__ignore_count <= 0:
                        print("Key Value Invalid")
            return self.STATE_OK
        except KeyboardInterrupt:
            if self.__debug:
                print('Key Break Joystick')
            return self.STATE_KEY_BREAK
        except:
            self.__js_isOpen = False
            print('---Joystick Disconnected---')
            return self.STATE_DISCONNECT

    # reconnect Joystick
    def reconnect(self):
        try:
            js = '/dev/input/js' + str(self.__js_id)
            self.__jsdev = open(js, 'rb')
            self.__js_isOpen = True
            self.__ignore_count = self.__IGNORE_MAX_COUNT
            print('---Opening %s Succeeded---' % js)
            return True
        except:
            self.__js_isOpen = False
            # if self.__debug:
            #     print('Failed To Open %s' % js)
            return False

    # 更新控制的小车类型 update cartype
    def update_cartype(self, car_type):
        self.__car_type = car_type


if __name__ == '__main__':
    g_debug = True
    if len(sys.argv) > 1:
        if str(sys.argv[1]) == "debug":
            g_debug = True
    print("debug=", g_debug)

    g_bot = Rosmaster(com='/dev/ttyUSB0', debug=g_debug)
    js = Rosmaster_Joystick(g_bot, debug=g_debug)
    try:
        while True:
            state = js.joystick_handle()
            if state != js.STATE_OK:
                if state == js.STATE_KEY_BREAK:
                    break
                time.sleep(1)
                js.reconnect()
    except KeyboardInterrupt:
        pass
    del js
