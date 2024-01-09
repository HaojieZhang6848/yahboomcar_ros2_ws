#!/usr/bin/env python3
# coding=utf-8
import os, struct, sys
from Rosmaster_Lib import Rosmaster
import time


# V1.1.0
class Rosmaster_Joystick(object):

    def __init__(self, robot:Rosmaster, js_id=0, debug=False):
        self.__debug = debug
        self.__js_id = int(js_id)
        self.__js_isOpen = False
        self.__robot = robot
        self.__ignore_count = 24

        self.__WIDTH_SCALE_X = 30
        self.__WIDTH_SCALE_Y = 0.2
        self.__WIDTH_SCALE_Z = 45.0

        self.__akm_angle = 0
        self.__car_back = False
        
        self.STATE_OK = 0
        self.STATE_NO_OPEN = 1
        self.STATE_DISCONNECT = 2
        self.STATE_KEY_BREAK = 3

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

    # Control robot
    def __data_processing(self, name, value):
        if name=="RK1_LEFT_RIGHT":
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            
        elif name == 'RK1_UP_DOWN':
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            fvalue = self.__WIDTH_SCALE_X * value/100.0
            if value >= 0:
                self.__robot.set_car_run(1, fvalue*100)
            else:
                self.__robot.set_car_run(2, -fvalue*100)

        elif name == 'RK2_LEFT_RIGHT':
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            fvalue = int(self.__WIDTH_SCALE_Z * -value)
            self.__akm_angle = fvalue
            self.__robot.set_akm_steering_angle(fvalue, True)

        elif name == 'RK2_UP_DOWN':
            value = value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            
        elif name == 'A':
            if self.__debug:
                print (name, ":", value)

        elif name == 'B':
            if self.__debug:
                print (name, ":", value)
            self.__akm_angle = self.__akm_angle + 2
            if self.__akm_angle > 44:
                self.__akm_angle = 44
            self.__robot.set_akm_steering_angle(self.__akm_angle)

        elif name == 'X':
            if self.__debug:
                print (name, ":", value)
            self.__akm_angle = self.__akm_angle - 2
            if self.__akm_angle < -44:
                self.__akm_angle = -44
            self.__robot.set_akm_steering_angle(self.__akm_angle)
            
        elif name == 'Y':
            if self.__debug:
                print (name, ":", value)
            
        elif name == 'L1':
            if self.__debug:
                print (name, ":", value)
            self.__WIDTH_SCALE_X = 50
            
        elif name == 'R1':
            if self.__debug:
                print (name, ":", value)
            self.__WIDTH_SCALE_X = 30
            
        elif name == 'SELECT':
            if self.__debug:
                print (name, ":", value)
            
        elif name == 'START':
            if self.__debug:
                print (name, ":", value)
            self.__robot.set_beep(value)
           
        elif name == 'MODE':
            if self.__debug:
                print (name, ":", value)
        elif name == 'BTN_RK1':
            if self.__debug:
                print (name, ":", value)
            
        elif name == 'BTN_RK2':
            if self.__debug:
                print (name, ":", value)
        
        elif name == "L2":
            value = ((value/32767)+1)/2
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            if int(value) == 1:
                self.__robot.set_car_motion(0, 0, 0)
                # self.__akm_angle = 0
                # self.__robot.reset_car_state()
            
        elif name == "R2":
            value = ((value/32767)+1)/2
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            if int(value) == 1:
                self.__robot.set_car_motion(0, 0, 0)
                # self.__akm_angle = 0
                # self.__robot.reset_car_state()
            
        elif name == 'WSAD_LEFT_RIGHT':
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            fvalue = (value * self.__WIDTH_SCALE_Y)
            
        elif name == 'WSAD_UP_DOWN':
            value = -value / 32767
            if self.__debug:
                print ("%s : %.3f" % (name, value))
            fvalue = int(value * self.__WIDTH_SCALE_X)
            if value == 0:
                if self.__car_back:
                    self.__car_back = False
                    self.__robot.set_car_motion(0, 0, 0)
                pass
            elif value > 0:
                self.__robot.set_car_run(1, fvalue)
            else:
                self.__car_back = True
                self.__robot.set_car_run(2, -fvalue)
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
                # print("evbuf:", timestamp, value, type, number)
                # if self.__debug:
                #     print("func:0x%04X, %s, %d" % (func, name, value))
                if name != None:
                    self.__data_processing(name, value)
                else:
                    if self.__ignore_count > 0:
                        self.__ignore_count = self.__ignore_count - 1
                    if self.__debug and self.__ignore_count == 0:
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
            self.__ignore_count = 24
            print('---Opening %s Succeeded---' % js)
            return True
        except:
            self.__js_isOpen = False
            # if self.__debug:
            #     print('Failed To Open %s' % js)
            return False


if __name__ == '__main__':
    g_debug = True
    if len(sys.argv) > 1:
        if str(sys.argv[1]) == "debug":
            g_debug = True
    print("debug=", g_debug)

    # g_bot = Rosmaster(com='/dev/ttyUSB0', debug=g_debug)
    g_bot = Rosmaster(debug=g_debug)
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
