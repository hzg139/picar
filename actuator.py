# -*- coding: UTF-8 -*-
import logging
import time
import RPi.GPIO as GPIO

class Actuator(object):
    '''使用Adafruit_Python_PCA968 '''
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    #默认针脚位(25,24,23,17)
    def __init__(self, ena=25, in1=24, in2=23, servopin=17):
        self.ENA = ena
        self.IN1 = in1
        self.IN2 = in2
        self.SERVOPIN = servopin

        GPIO.setup(self.ENA, GPIO.OUT, initial=GPIO.LOW)
        # 利用PWM改变转速 初始值为100
        self.ENA_SPEED = GPIO.PWM(self.ENA, 600)
        self.ENA_SPEED.start(0)
        self.ENA_SPEED.ChangeDutyCycle(100)
        GPIO.setup(self.IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN2, GPIO.OUT, initial=GPIO.LOW)
        # 设置舵机
        GPIO.setup(self.SERVOPIN, GPIO.OUT)
        self.servo = GPIO.PWM(self.SERVOPIN, 100)
        self.servo.start(0.0)

    #Motor速度
    def speed(self, speed):
        self.ENA_SPEED.ChangeDutyCycle(speed)

    #前进
    def forward(self):
        GPIO.output(self.ENA, True)
        GPIO.output(self.IN1, True)
        GPIO.output(self.IN2, False)

    #后退
    def backward(self):
        GPIO.output(self.ENA, True)
        GPIO.output(self.IN1, False)
        GPIO.output(self.IN2, True)

    #舵机转向 默认90为中置
    def steer(self,angle=90):
        self.servo.ChangeDutyCycle(calculate_angle2dc(angle))

    def stop(self):
        GPIO.output(self.ENA, True)
        GPIO.output(self.IN1, False)
        GPIO.output(self.IN2, False)
        self.servo.stop()
        GPIO.cleanup()

#转换舵机角度为ChangeDutyCycle值，存在1.5偏差，各舵机情况不同
def calculate_angle2dc(angle):
    return ((float(angle) * 0.01) + 0.5) * 10 + 1.5


def test():
    motor = Actuator()
    motor_speed = 20
    servo_angle = 90
    while True:
        motor.speed(motor_speed)
        motor.steer(servo_angle)
        #motor.steer(15.5)
        key = input()
        if key == 'f':
            logging.info('forward')
            motor.forward()
        elif key == 'b':
            logging.info('backward')
            motor.backward()
        elif key == 'r':
            logging.info('turn right')
            servo_angle += 10
            if servo_angle >=180 :
                servo_angle = 180
        elif key == 'l':
            logging.info('turn left')
            servo_angle -= 10
            if servo_angle <= 0:
                servo_angle = 0
        elif key == 'm':
            logging.info('low speed')
            motor_speed -= 10
            if motor_speed <= 20:
                motor_speed = 20
        elif key == 'h':
            logging.info('high speed')
            motor_speed += 10
            if motor_speed >= 100:
                motor_speed = 100
        elif key == 's':
            logging.info('stop')
            motor.stop()
        elif key == 'e':
            logging.info('exit')
            motor.stop()
            break

if __name__ == '__main__':
	test()