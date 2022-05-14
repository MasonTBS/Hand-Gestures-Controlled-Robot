import socket 
import threading
import RPi.GPIO as GPIO
import time

HEADER = 64
PORT = 5050
SERVER = socket.gethostbyname(socket.gethostname())
ADDR = (SERVER, PORT)
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(ADDR)


leftSpeed = 0.0
rightSpeed = 0.0

left_pwmpin = 13
right_pwmpin = 14
GPIO.setmode(GPIO.BCM)
GPIO.setup(left_pwmpin, GPIO.OUT)
GPIO.setup(right_pwmpin, GPIO.OUT)
left_pwm = GPIO.PWM(left_pwmpin, 50)
right_pwm = GPIO.PWM(right_pwmpin, 50)
left_pwm.start(0)
right_pwm.start(0)




def handle_client(conn, addr):
    print(f"[NEW CONNECTION] {addr} connected.")

    connected = True
    while connected:
        msg_length = conn.recv(HEADER).decode(FORMAT)
        if msg_length:
            msg_length = int(msg_length)
            msg = conn.recv(msg_length).decode(FORMAT)
            if msg == DISCONNECT_MESSAGE:
                connected = False
            leftSpeed, rightSpeed = msg.split(',')

            
            left_pwm.start(100*float(leftSpeed))
            right_pwm.start(100*float(rightSpeed))


            print (leftSpeed, rightSpeed)
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

start()
while(True):
    loop()

