
import socket
#importing time
import time
#importing Serial to take data from serial port
import serial
#importing the ke yboard listener from pynput
from pynput import keyboard
import numpy as np
##################################################################################
###################### SOCKET OBJECT AND VARIABLES ###################################################
##################################################################################

commands = []    #list of commands

s = socket.socket()
host = '192.168.10.102'  #IP Address of the Raspberry pi
port = 9999            #Must be same as that in server.py
#In client.py we use another way to bind host and port together by using connect function()
s.connect((host, port))
print('connected to the host')
mode = 0;   #0-> Propulsion
forwardBackwardSpeed = 0;
leftRightSpeed = 0;
linear_velocity=0;
angular_velocity=0;
width_chassi=0;
radius_wheel=0;
def sendDatatoRaspi():
    global forwardBackwardSpeed, leftRightSpeed;

        stringData = str(mode) + ',' + str(forwardBackwardSpeed) + ',' + str(leftRightSpeed)
        s.send(str.encode(stringData))
    # After sending we check if it was recieved or not
    checkDataTranfer = s.recv(1024)
    print(checkDataTranfer)

def printSpeeds():
    global forwardBackwardSpeed, leftRightSpeed;

        stringData = str(mode) + ',' + str(forwardBackwardSpeed) + ',' + str(leftRightSpeed)
        print("MODE 0 DATA :",stringData)

def generate_commands():
    global forwardBackwardSpeed,linear_velocity,angular_velocity,leftRightSpeed,width_chassi,radius_wheel;
    forwardBackwardSpeed = linear_velocity/radius_wheel
    leftRightSpeed = (angular_velocity*width_chassi)/(2*radius_wheel)
    print("generated forwardBackwardSpeed and leftRightSpeed")


#generate linear velocity and angular velocity from path planning module
#now one more question is left is to how to genrate linear and angular velocity from vx and vy
    
