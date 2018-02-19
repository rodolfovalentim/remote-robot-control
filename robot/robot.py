import RPi.GPIO as gpio
import socket
import json


def init():
    gpio.setmode(gpio.BCM)
    gpio.setup(18, gpio.OUT)
    gpio.setup(27, gpio.OUT)
    gpio.setup(17, gpio.OUT)
    gpio.setup(22, gpio.OUT)
    gpio.setup(23, gpio.OUT)
    gpio.setup(24, gpio.OUT)
    pwm_dir = gpio.PWM(27, 50)
    pwm_dir.start(0)
    pwm_esq = gpio.PWM(18, 50)
    pwm_esq.start(0)
    return pwm_esq, pwm_dir


UDP_IP = "192.168.1.114"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

while True:
    pwm_dir = None
    pwm_esq = None
    try:
        data, addr = sock.recvfrom(1024)
        print "received message:", data
        speed = json.loads(data)
        pwm_esq, pwm_dir = init()
        pwm_dir.ChangeDutyCycle(int(speed['vr']))
        pwm_esq.ChangeDutyCycle(int(speed['vl']))
        gpio.output(17, False)
        gpio.output(22, True)
        gpio.output(23, False)
        gpio.output(24, True)
    except KeyboardInterrupt:
        print "Keyboard Interrupt"
        if pwm_dir is not None:
            pwm_dir.stop()
        if pwm_esq is not None:
            pwm_esq.stop()
        gpio.cleanup()