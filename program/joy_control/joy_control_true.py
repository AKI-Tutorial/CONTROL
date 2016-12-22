#!/usr/bin/env python
''' Joy stick controller

@author Kyohei Otsu <kyon@ac.jaxa.jp>
@date   2015-07-15

Usage:
'''

import logging
import sys
import socket
import struct
import time

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

logfile = "cmd_log_{}.txt".format(time.time())
#logging.basicConfig(level=logging.DEBUG, filename=logfile)
logging.basicConfig(level=logging.DEBUG)
#logging.info("Log start for {}".format(logfile))


## BUTTON ID
BUTTON_SEL = 0
BUTTON_STR = 3

BUTTON_U = 4
BUTTON_R = 5
BUTTON_D = 6
BUTTON_L = 7

BUTTON_UL2 = 8
BUTTON_UR2 = 9
BUTTON_UL1 = 10
BUTTON_UR1 = 11

BUTTON_TR = 12
BUTTON_CI = 13
BUTTON_CR = 14
BUTTON_SQ = 15

BUTTON_PS = 16


## MODE SPECIFIER
MODE_4WD = 0
MODE_2WD = 1
MODE_SPOTTURN = 2

## KEY UPDATE FLAG
KEY_NOEVENT = 0
KEY_PUSHED  = 1

## STR Angles
STR_CNT_INC = 1
STR_CNT_LMT = 15

## PAN/TILT Angles
PTL_CNT_INC = 0.1
PTL_CNT_LMT = 90


class JoyController:

    def __init__(self):
        self.cmd_list = []
        self.mode = MODE_4WD
        self.flag_moving = False
        self.flag_rotating = False
        self.flag_initial = True
        self.str_angle = 0
        self.pan_angle = 0
        self.tilt_angle = 0


    def checkDPad(self, btn):
        ''' Check the input from direction keypad 

        Auguments:
            btn: button status from joy message

        Returns:
            Status code
            Command to send in self.cmd_list
        '''

        # Do not use dpad while turning in spot
        if (self.mode == MODE_SPOTTURN): return KEY_NOEVENT

        # Clear command list
        self.cmd_list = []

        # Count number of pressed buttons
        num_pushed = sum(btn[i] for i in [BUTTON_U, BUTTON_R, BUTTON_D, BUTTON_L])

        if (num_pushed == 0):
            # Key released. Restore initial mode
            if (self.flag_moving or self.str_angle != 0):
                self.flag_moving = False
                self.cmd_list.append("f")
                self.str_angle = 0

        elif (num_pushed == 1 or num_pushed == 2):
            # She cannot move like this
            if (btn[BUTTON_U] and btn[BUTTON_D]): return KEY_NOEVENT
            if (btn[BUTTON_L] and btn[BUTTON_R]): return KEY_NOEVENT

            # Send drive command forward
            if (btn[BUTTON_U] and not self.flag_moving):
                self.cmd_list.append("d30")
                self.flag_moving = True

            # Send drive command backword
            if (btn[BUTTON_D] and not self.flag_moving):
                self.cmd_list.append("d-30")
                self.flag_moving = True 

            # Rotate steer left
            if (btn[BUTTON_L]):
                self.str_angle += STR_CNT_INC
                if (self.str_angle > STR_CNT_LMT):
                    self.str_angle = STR_CNT_LMT
                    # do not send command anymore
                elif (self.str_angle % 5 < STR_CNT_INC):
                    if (self.mode == MODE_4WD):
                        self.cmd_list.append("u{:.2f}".format(-self.str_angle))
                    else:
                        self.cmd_list.append("s{:.2f}".format(-self.str_angle))

            # Rotate steer right
            if (btn[BUTTON_R]):
                self.str_angle -= STR_CNT_INC
                if (self.str_angle < -STR_CNT_LMT):
                    self.str_angle = -STR_CNT_LMT
                    # do not send command anymore
                elif (self.str_angle % 5 < STR_CNT_INC):
                    if (self.mode == MODE_4WD):
                        self.cmd_list.append("u{:.2f}".format(-self.str_angle))
                    else:
                        self.cmd_list.append("s{:.2f}".format(-self.str_angle))

            # Drive command released
            if (not btn[BUTTON_U] and not btn[BUTTON_D] and self.flag_moving):
                self.flag_moving = False
                self.cmd_list.append("d0")

            # Steer command released
            if (not btn[BUTTON_L] and not btn[BUTTON_R] and self.str_angle != 0):
                self.str_angle = 0
                if (self.mode == MODE_4WD):
                    self.cmd_list.append("u{:.2f}".format(-self.str_angle))
                else:
                    self.cmd_list.append("s{:.2f}".format(-self.str_angle))


        if (len(self.cmd_list) > 0): return KEY_PUSHED
        return KEY_NOEVENT


    def checkFrontKey(self, btn):
        ''' Check the input from front buttons'''

        # Clear command list
        self.cmd_list = []

        # Count number of pressed buttons
        num_pushed = sum(btn[i] for i in [BUTTON_TR, BUTTON_CI, BUTTON_CR, BUTTON_SQ, BUTTON_PS])

        if (num_pushed == 0): return KEY_NOEVENT

        elif (num_pushed == 1):
            # PS button pressed
            if (btn[BUTTON_PS] and 
                    (self.mode == MODE_SPOTTURN or self.pan_angle != 0 or self.tilt_angle != 0)):
                self.cmd_list.append("f")
                self.mode = MODE_4WD
                self.pan_angle = 0
                self.tilt_angle = 0

            # Pan
            if (btn[BUTTON_CI]): 
                self.pan_angle += PTL_CNT_INC
                if (self.pan_angle > PTL_CNT_LMT):
                    self.pan_angle = PTL_CNT_LMT
                    # do not send command anymore
                elif (self.pan_angle % 5 < PTL_CNT_INC):
                    self.cmd_list.append("a{:.2f}".format(self.pan_angle))

            if (btn[BUTTON_SQ]): 
                self.pan_angle -= PTL_CNT_INC
                if (self.pan_angle < -PTL_CNT_LMT):
                    self.pan_angle = -PTL_CNT_LMT
                    # do not send command anymore
                elif (self.pan_angle % 5 < PTL_CNT_INC):
                    self.cmd_list.append("a{:.2f}".format(self.pan_angle))

            # Tilt
            if (btn[BUTTON_TR]): 
                self.tilt_angle += PTL_CNT_INC
                if (self.tilt_angle > PTL_CNT_LMT):
                    self.tilt_angle = PTL_CNT_LMT
                    # do not send command anymore
                elif (self.tilt_angle % 1 < PTL_CNT_INC):
                    self.cmd_list.append("o{:.2f}".format(-self.tilt_angle))

            if (btn[BUTTON_CR]): 
                self.tilt_angle -= PTL_CNT_INC
                if (self.tilt_angle < -PTL_CNT_LMT):
                    self.tilt_angle = -PTL_CNT_LMT
                    # do not send command anymore
                elif (self.tilt_angle % 1 < PTL_CNT_INC):
                    self.cmd_list.append("o{:.2f}".format(-self.tilt_angle))


        elif (num_pushed == 2): pass

        if (len(self.cmd_list) > 0): return KEY_PUSHED
        return KEY_NOEVENT


    def checkRearKey(self, btn):

        # Clear command list
        self.cmd_list = []

        # Count number of pressed buttons
        num_pushed = sum(btn[i] for i in [BUTTON_UL1, BUTTON_UL2, BUTTON_UR1, BUTTON_UR2])
        if (num_pushed == 0): 
            # No button pressed
            if (self.mode == MODE_SPOTTURN and self.flag_rotating):
                self.cmd_list.append("e")
                self.flag_rotating = False

        elif (num_pushed == 1):
            if (self.mode == MODE_SPOTTURN):
                # Rotate left
                if (btn[BUTTON_UL2] and not self.flag_rotating):
                    self.cmd_list.append("r-180")
                    self.flag_rotating = True

                # Rotate right
                if (btn[BUTTON_UR2] and not self.flag_rotating):
                    self.cmd_list.append("r180")
                    self.flag_rotating = True

        elif (num_pushed == 2):
            if (self.mode != MODE_SPOTTURN):
                if (btn[BUTTON_UL1] and btn[BUTTON_UR1]):
                    self.mode = MODE_SPOTTURN
                    self.cmd_list.append("i")
                    self.flag_initial = False

            if (btn[BUTTON_UL2] and btn[BUTTON_UR2]):
                if (self.mode == MODE_SPOTTURN):
                    self.cmd_list.append("e")
                    self.flag_rotating = False


        if (len(self.cmd_list) > 0): return KEY_PUSHED
        return KEY_NOEVENT


controller = JoyController()
sock = None
cmd_seq = 0


def callback(msg):
    global controller
    if (controller.checkDPad(msg.buttons) == KEY_PUSHED): pass
    elif (controller.checkFrontKey(msg.buttons) == KEY_PUSHED): pass
    elif (controller.checkRearKey(msg.buttons) == KEY_PUSHED): pass
    send_cmd(controller.cmd_list)


def serialize(hexstr):
    ''' Convert hex string into byte lists '''
    assert len(hexstr) % 2 == 0

    serstr = ''
    for h in [hexstr[i:i+2] for i in range(0, len(hexstr), 2)]:
        serstr += chr(int(h, base=16))
    return serstr


def capsulate(cmd):
    ''' Append header and convert to byte strings'''
    global cmd_seq
    cmd_seq += 1

    marker = '0020f3fa00000000'
    sender_id = '0039'
    seq = '{:04d}'.format(cmd_seq)
    cmd_type = cmd[0]
    packet = None
    if len(cmd) > 1:
        cmd_arg = struct.pack('f', float(cmd[1:]))
    else:
        cmd_arg = struct.pack('f', float(0))
    packet = serialize(marker + sender_id + seq) + cmd_type + cmd_arg
    #print(marker + sender_id + seq) + cmd_type + cmd_arg
    return packet


def send_cmd(cmd_list):
    global sock
    for c in cmd_list:
        print time.time(), c
        logging.info(c)
        try:
            sock.sendall(capsulate(c))
        except:
            sock = connect()



def connect():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect(("192.168.201.11", 13000))
        print "Connected"
    except:
        print "connect() timeout"
        sock = None
    return sock


if __name__ == "__main__":
    try:
        rospy.init_node("joy_control", anonymous=True)
        joy_sub = rospy.Subscriber("joy", Joy, callback)
        connect()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
    sock.close()
    print 'Session closed'

