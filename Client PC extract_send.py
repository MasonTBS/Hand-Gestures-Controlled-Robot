import cv2  # OpenCV
import mediapipe  # For the hand module
import numpy  # Good Old Numpy
import time  # To track the time
import socket

HEADER = 64
PORT = 5050
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
SERVER = "192.168.43.197"
ADDR = (SERVER, PORT)

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(ADDR)

print("connection_established")

# CONSTANTS
DISCONNECT_MESSAGE = "!!"
motorSpeed = 1
thumbTipId = 4
indexTipId = 8
palmId = 0
usableWidth = 640
xMax = 230

# Variables initialization
t_FPS_prev = 0  # For counting FPS: Frames per Second
leftRatio = 0.0
rightRatio = 0.0
rightSpeed = 0.0
leftSpeed = 0.0
speedRatio = 0.0
mode = 1
lastHand = 1
checkHand = 1

# cap2 = cv2.VideoCapture('http://192.168.1.54:8080/video')  # Defining the USB camera
cap = cv2.VideoCapture(0)  # Defining the Video Recording device(Webcam)
initHand = mediapipe.solutions.hands  # Initializing mediapipe
# Object of mediapipe with "arguments for the hands module"
mainHand = initHand.Hands(max_num_hands=1, min_detection_confidence=0.5, min_tracking_confidence=0.5)
draw = mediapipe.solutions.drawing_utils  # Object to draw the connections between each finger index


def handLandmarks(colorImg):
    landmarkList = []  # Default values if no landmarks are tracked
    landmarkPositions = mainHand.process(colorImg)  # Object for processing the video input
    landmarkCheck = landmarkPositions.multi_hand_landmarks  # Stores the out of the processing object (returns False on empty)
    if landmarkCheck:  # Checks if landmarks are tracked
        for hand in landmarkCheck:  # Landmarks for each hand
            for index, landmark in enumerate(
                    hand.landmark):  # Loops through the 21 indexes and outputs their landmark coordinates (x, y, & z)
                draw.draw_landmarks(img, hand,
                                    initHand.HAND_CONNECTIONS)  # Draws each individual index on the hand with connections
                h, w, c = img.shape  # Height, width and channel on the image
                centerX, centerY = int(landmark.x * w), int(
                    landmark.y * h)  # Converts the decimal coordinates relative to the image for each index
                landmarkList.append([index, centerX, centerY])  # Adding index and its coordinates to a list
    return landmarkList

def fingers(landmarks):
    # returns the state of fingers! 1 if up and 0 if down
    fingerTips = []  # To store 4 sets of 1s or 0s
    tipIds = [4, 8, 12, 16,
              20]  # Indexes for the tips of each finger (Check for visulization: https://google.github.io/mediapipe/solutions/hands.html)

    # Check if thumb is up
    if landmarks[tipIds[0]][1] > lmList[tipIds[0] - 1][1]:
        fingerTips.append(1)
    else:
        fingerTips.append(0)
    # Check if fingers are up except the thumb
    for id in range(1, 5):
        if landmarks[tipIds[id]][2] < landmarks[tipIds[id] - 3][2]:
            # Checks to see if the tip of the finger is higher than the joint
            fingerTips.append(1)
        else:
            fingerTips.append(0)
    return fingerTips


def calculate_FPS(t_FPS_prev):
    t_FPS = time.time()
    FPS = 1 / (t_FPS - t_FPS_prev)  # Calculate FPS
    t_FPS_prev = t_FPS
    return FPS, t_FPS_prev


def caluculateSpeedRatio():
    # CHANGE THE DISTANCE BETWEEN YOUR THUMB AND INDEX TO CHANGE THE SPEED RATIO
    thumbTip = numpy.array(lmList[thumbTipId])
    indexTip = numpy.array(lmList[indexTipId])
    distance = numpy.linalg.norm(thumbTip-indexTip)
    ratio = min(max(distance - 30, 0), 100)/100
    return ratio


def calculateTurningRatio():
    # MOVE YOUR THUMB RIGHT AND LEFT HORIZONTALLY TO CHANGE LEFT AND RIGHT SPEED RATIOS
    xMid = usableWidth / 2
    palm = lmList[palmId]
    x = palm[1]

    if (x < xMid):
        rightRatio = 1
        leftRatio = max(-1, 1 + 2 * (x - xMid) / xMax)
    else:
        rightRatio = max(-1, 1 - 2 * (x - xMid) / xMax)
        leftRatio = 1
    return leftRatio, rightRatio


def calculateMode(mode):
    if(lastHand == 1 and checkHand == 0):
        return mode * -1
    return mode

def checkHandState(finger):
    if(finger[0] == finger[1] == finger[2] == finger[3] == finger[4]==0):
        return 0
    return 1

def send(msg):
    message = msg.encode(FORMAT)
    msg_length = len(message)
    send_length = str(msg_length).encode(FORMAT)
    send_length += b' ' * (HEADER - len(send_length))
    client.send(send_length)
    client.send(message)

while True:
    # Calculate FPS
    FPS, t_FPS_prev = calculate_FPS(t_FPS_prev)
    # Reads frames from the camera
    check, img = cap.read()
    # check2, img2 = cap2.read()
    # Changes the format of the frames from BGR to RGB
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    # Print a pink rectangle that limits the uable mouse region
    cv2.rectangle(img, (75, 0), (usableWidth - 75, 480 - 150), (255, 0, 0),2)
    # Stores the Hand Landmark result
    lmList = handLandmarks(imgRGB)

    # Extracting Information from hand landmarks
    if len(lmList) != 0:  # Check to see if any fingers are detected
        finger = fingers(lmList)
        speedRatio = caluculateSpeedRatio()
        leftRatio, rightRatio = calculateTurningRatio()
        rightSpeed = speedRatio * rightRatio
        leftSpeed = speedRatio * leftRatio
        checkHand = checkHandState(finger)
        mode = calculateMode(mode)
        message = str(leftSpeed) + ',' + str(rightSpeed)+','+ str(mode)
    else:
        message = "0,0,"+str(mode)
    # print(message)
    send(message)
    time.sleep(0.01)

    # Video Display
    img = cv2.flip(img, 1)
    cv2.putText(img, str(round(FPS)), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, [0, 150, 0], 3)  # Prints FPS
    img = cv2.resize(img, (1280, 480))
    # img2 = cv2.resize(img2, (360, 240))
    cv2.imshow("Webcam", img)
    # cv2.imshow("camera", img2)
    if cv2.waitKey(10) & 0xFF == ord('q'):  # Closes the window if Q is pressed
        send(DISCONNECT_MESSAGE)
        break
    lastHand = checkHand

cap.release()  # Releasing the video recording device
cv2.destroyAllWindows()  # Destroying all the opened windows









