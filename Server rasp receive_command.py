import socket
import threading
import RPi.GPIO as GPIO
import time

leftFrontPin = 5
leftBackPin = 6
rightFrontPin = 26
rightBackPin = 19
left_pwmpin = 13
right_pwmpin = 12

leftSpeed = 0.0
rightSpeed = 0.0
mode = 1

HEADER = 64
PORT = 5050
SERVER = "192.168.43.197"
ADDR = (SERVER, PORT)
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(ADDR)

GPIO.setmode(GPIO.BCM)

GPIO.setup(rightFrontPin, GPIO.OUT)
GPIO.setup(leftFrontPin, GPIO.OUT)
GPIO.setup(rightBackPin, GPIO.OUT)
GPIO.setup(leftBackPin, GPIO.OUT)

GPIO.setmode(GPIO.BCM)
GPIO.setup(left_pwmpin, GPIO.OUT)
GPIO.setup(right_pwmpin, GPIO.OUT)
left_pwm = GPIO.PWM(left_pwmpin, 50)
right_pwm = GPIO.PWM(right_pwmpin, 50)
left_pwm.start(0)
right_pwm.start(0)


def handle_client(conn, addr):
    print(f"[NEW CONNECTION] {addr} connected.")

    while True:
        msg_length = conn.recv(HEADER).decode(FORMAT)
        if msg_length:
            msg_length = int(msg_length)
            msg = conn.recv(msg_length).decode(FORMAT)
            if msg == DISCONNECT_MESSAGE:
                break
            leftMsg, rightMsg, modeMsg = msg.split(',')
            rightSpeed = 100 * float(leftMsg)
            leftSpeed = 100 * float(rightMsg)
            mode = int(modeMsg)

            setMotors(mode, leftSpeed, rightSpeed)
    conn.shutdown(socket.SHUT_RDWR)
    conn.close()


def start():
    print("[STARTING] server is starting...")
    server.listen()
    print(f"[LISTENING] Server is listening on {SERVER}")


def loop():
    conn, addr = server.accept()
    thread = threading.Thread(target=handle_client, args=(conn, addr))
    thread.start()

def setRightDirection(mode):
    if(mode==1): # front
        GPIO.output(rightFrontPin, True)
        GPIO.output(rightBackPin, False)
    else:
        GPIO.output(rightFrontPin, False)
        GPIO.output(rightBackPin, True)

def setLeftDirection(mode):
    if(mode==1): # front
        GPIO.output(leftFrontPin, True)
        GPIO.output(leftBackPin, False)
    else:
        GPIO.output(leftFrontPin, False)
        GPIO.output(leftBackPin, True)


def setMotors(mode, leftSpeed, rightSpeed):
    if(leftSpeed < 0):
        leftMode = 1
        rightMode = -1
    elif(rightSpeed < 0):
        leftMode = -1
        rightMode = 1
    else:
        leftMode = mode
        rightMode = mode
    setLeftDirection(leftMode)
    setRightDirection(rightMode)
    left_pwm.start(abs(leftSpeed))
    right_pwm.start(abs(rightSpeed))

    print(mode)

start()
while (True):
    loop()







