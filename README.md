# Hand-Gestures-Controlled-Robot
This project was done using Raspberry Pi card, computer vision and a four-wheel robot.
the code is composed of two parts:
The first code(python) is for PC (Client) which detects user's hand and extracts useful info(wheels speed/mode) from hand landmarks(opencv/mediapie),
and then sends these information to raspberry using "Socket" library(connect pc and raspberry to the same local network).
The second code(python)is for Raspberry card (Server) receives information from the pc using the same library and set the motor speed and direction according to them.
The Raspberry pi card is powered inside the robot by a powerbank.
The motors are powered by LI-PO batteries. 
