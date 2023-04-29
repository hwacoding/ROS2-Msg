#!/usr/bin/env python
from __future__ import division

import RPi.GPIO as GPIO
import time

import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)

#  서보모터의 펄스 길이를 최소, 중간, 최대로 설정
# servo_min = 130  # Min pulse length out of 4096

servo_open =  255 # Middle pulse length out of 4096  ##390 
servo_close = 570  # Max pulse length out of 4096  ##650

pwm.set_pwm_freq(60)

pwm.set_pwm(0, 0, servo_open)

# for 1st Motor on ENA
ENA = 37
IN1 = 35
IN2 = 33

# # set pin numbers to the board's
GPIO.setmode(GPIO.BOARD)

# initialize EnA, In1 and In2
GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)

# Stop
GPIO.output(ENA, GPIO.HIGH)
GPIO.output(IN1, GPIO.LOW)
GPIO.output(IN2, GPIO.LOW)
time.sleep(1)

# Forward
GPIO.output(IN1,GPIO.HIGH)
GPIO.output(IN2, GPIO.LOW)
time.sleep(3)

# Stop
GPIO.output(IN1, GPIO.LOW)
GPIO.output(IN2, GPIO.LOW)
time.sleep(1)

pwm.set_pwm(0, 0, servo_close)

GPIO.cleanup()
