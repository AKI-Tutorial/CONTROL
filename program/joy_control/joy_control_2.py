#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

import sys


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
STR_CNT_INC = 2
STR_CNT_LMT = 15


class JoyController:
  def __init__(self):
    self.cmd_list = []
    self.str_cnt = 0
    self.mode = MODE_4WD
    self.flag_moving = False
    self.flag_rotating = False



  def checkDPad(self, btn):
    if (self.mode == MODE_SPOTTURN): return KEY_NOEVENT

    self.cmd_list = []

    num_pushed = sum(btn[i] for i in [BUTTON_U, BUTTON_R, BUTTON_D, BUTTON_L])
    if (num_pushed == 0):
      if (self.flag_moving or self.str_cnt != 0):
        self.flag_moving = False
        self.cmd_list.append("f")
        self.str_cnt = 0
    elif (num_pushed == 1 or num_pushed == 2):
      if (btn[BUTTON_U] and btn[BUTTON_D]): return KEY_NOEVENT
      if (btn[BUTTON_L] and btn[BUTTON_R]): return KEY_NOEVENT

      if (btn[BUTTON_U] and not self.flag_moving):
        self.cmd_list.append("d30")
        self.flag_moving = True
      if (btn[BUTTON_D] and not self.flag_moving):
        self.cmd_list.append("d-30")
        self.flag_moving = True 
      if (btn[BUTTON_L]):
        self.str_cnt += STR_CNT_INC
        self.str_cnt = min(self.str_cnt, STR_CNT_LMT)
        if (self.str_cnt != STR_CNT_LMT):
          if (self.mode == MODE_4WD):
            self.cmd_list.append("u" + repr(self.str_cnt))
          else:
            self.cmd_list.append("s" + repr(self.str_cnt))
      if (btn[BUTTON_R]):
        self.str_cnt -= STR_CNT_INC
        self.str_cnt = max(self.str_cnt, -STR_CNT_LMT)
        if (self.str_cnt != -STR_CNT_LMT):
          if (self.mode == MODE_4WD):
            self.cmd_list.append("u" + repr(self.str_cnt))
          else:
            self.cmd_list.append("s" + repr(self.str_cnt))
    if (len(self.cmd_list) > 0): return KEY_PUSHED
    return KEY_NOEVENT

  def checkFrontKey(self, btn):
    self.cmd_list = []
    num_pushed = sum(btn[i] for i in [BUTTON_TR, BUTTON_CI, BUTTON_CR, BUTTON_SQ, BUTTON_PS])
    if (num_pushed == 0): return KEY_NOEVENT
    elif (num_pushed == 1):
      if (btn[BUTTON_PS]):
        self.cmd_list.append("f")
        self.mode = MODE_4WD
    elif (num_pushed == 2): pass

    if (len(self.cmd_list) > 0): return KEY_PUSHED
    return KEY_NOEVENT

  def checkRearKey(self, btn):
    self.cmd_list = []
    num_pushed = sum(btn[i] for i in [BUTTON_UL1, BUTTON_UL2, BUTTON_UR1, BUTTON_UR2])
    if (num_pushed == 0): 
      if (self.mode == MODE_SPOTTURN and self.flag_rotating):
        self.cmd_list.append("e")
        self.flag_rotating = False
    elif (num_pushed == 1):
      if (self.mode == MODE_SPOTTURN):
        if (btn[BUTTON_UL2] and not self.flag_rotating):
          self.cmd_list.append("r180")
          self.flag_rotating = True
        if (btn[BUTTON_UR2] and not self.flag_rotating):
          self.cmd_list.append("r-180")
          self.flag_rotating = True
    elif (num_pushed == 2):
      if (self.mode != MODE_SPOTTURN):
        if (btn[BUTTON_UL1] and btn[BUTTON_UR1]):
          self.mode = MODE_SPOTTURN
          self.cmd_list.append("i")
    if (len(self.cmd_list) > 0): return KEY_PUSHED
    return KEY_NOEVENT

controller = JoyController()


def callback(msg):
  global controller
  if (controller.checkDPad(msg.buttons) == KEY_PUSHED): pass
  elif (controller.checkFrontKey(msg.buttons) == KEY_PUSHED): pass
  elif (controller.checkRearKey(msg.buttons) == KEY_PUSHED): pass
  send_cmd(controller.cmd_list)

def send_cmd(cmd_list):
  for c in cmd_list:
    cmd_pub.publish(c)


if __name__ == "__main__":
  try:
    rospy.init_node("joy_control", anonymous=True)
    joy_sub = rospy.Subscriber("joy", Joy, callback)
    cmd_pub = rospy.Publisher("command", String)
    rospy.spin()
  except rospy.ROSInterruptException: pass
