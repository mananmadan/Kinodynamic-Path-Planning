##################################################################################
########################IMPORTING LIBRARIES ######################################
##################################################################################
#importing socket so that we can connect two computer
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
print('hello1')
#In client.py we use another way to bind host and port together by using connect function()
s.connect((host, port))
print('hello2')
###########################SERIAL OBJECT ##############################################
# serialPortMac = '/dev/tty.usbmodem14101' #FOR MACBOOK
# serialPortWin = '/dev/ttyUSB0'           #FOR WINDOWS
# serialPortUbuntu = '/dev/ttyACM0'        #FOR UBUNTU
# ser = serial.Serial(serialPortUbuntu, 9600,timeout=0.005)
#######################################################################################
mode = 0;   #0-> Propulsion, 1-> Robotic Arm
# PROPULSION VARIABLES
forwardBackwardSpeed = 0;
leftRightSpeed = 0;
deltaIncrement = 1;
brakeValue = 3;
#ROBOTIC ARM VARAIBLES:
baseMotor = 0;
baseActuator = 0;
armActuator = 0;
clawPitch = 0;
clawRoll = 0;
clawOpenClose = 0;

def sendDatatoRaspi():
    global forwardBackwardSpeed, leftRightSpeed;
    global baseMotor, baseActuator, armActuator, clawRoll, clawPitch, clawOpenClose;
    global mode

    if(mode == 0):
        stringData = str(mode) + ',' + str(forwardBackwardSpeed) + ',' + str(leftRightSpeed)
        s.send(str.encode(stringData))
    elif(mode == 1):
        stringData = str(mode) + ',' + str(baseMotor) + ',' + str(baseActuator) + ',' + str(armActuator) + ',' + str(clawPitch) + ',' + str(clawRoll)  + ',' + str(clawOpenClose);
        s.send(str.encode(stringData))
    # After sending we check if it was recieved or not
    checkDataTranfer = s.recv(1024)
    print(checkDataTranfer)

##################################################################################
################# Variables amd functions for PROPULSION SPEED ###################
##################################################################################
def increaseForwardBackwardSpeed():
    global forwardBackwardSpeed
    global deltaIncrement
    forwardBackwardSpeed = forwardBackwardSpeed + deltaIncrement;
    if(forwardBackwardSpeed > 100):
        forwardBackwardSpeed = 100;
    printSpeeds()
    sendDatatoRaspi()

def decreaseForwardBackwardSpeed():
    global forwardBackwardSpeed
    global deltaIncrement
    forwardBackwardSpeed = forwardBackwardSpeed - deltaIncrement;
    if (forwardBackwardSpeed < -100):
        forwardBackwardSpeed = -100;
    printSpeeds()
    sendDatatoRaspi()

def increaseleftRightSpeed():
    global leftRightSpeed
    global deltaIncrement
    leftRightSpeed = leftRightSpeed + deltaIncrement;
    if (leftRightSpeed > 100):
        leftRightSpeed = 100;
    printSpeeds()
    sendDatatoRaspi()

def decreaseleftRightSpeed():
    global leftRightSpeed
    global deltaIncrement
    leftRightSpeed = leftRightSpeed - deltaIncrement;
    if (leftRightSpeed < -100):
        leftRightSpeed = -100;
    printSpeeds()
    sendDatatoRaspi()

def stopAllMotors():
    global forwardBackwardSpeed, leftRightSpeed;
    global baseMotor, baseActuator, armActuator, clawRoll, clawPitch, clawOpenClose;
    leftRightSpeed = 0;
    forwardBackwardSpeed = 0;
    baseMotor = 0;
    baseActuator = 0;
    armActuator = 0;
    clawRoll = 0;
    clawPitch = 0;
    clawOpenClose = 0;
    printSpeeds();
    sendDatatoRaspi();

def applyBrakes():
    global forwardBackwardSpeed, leftRightSpeed;
    global baseMotor, baseActuator, armActuator, clawRoll, clawPitch, clawOpenClose;
    global brakeValue;
    print('applying brakes');
    if(mode == 0):
        if(forwardBackwardSpeed > 5):
            forwardBackwardSpeed = forwardBackwardSpeed - brakeValue;
        elif(forwardBackwardSpeed < -5):
            forwardBackwardSpeed = forwardBackwardSpeed + brakeValue;
        else:
            forwardBackwardSpeed = 0;

        if(leftRightSpeed > 5):
            leftRightSpeed = leftRightSpeed - brakeValue;
        elif(leftRightSpeed < -5):
            leftRightSpeed = leftRightSpeed + brakeValue;
        else:
            leftRightSpeed = 0;
        printSpeeds();
        sendDatatoRaspi();

def moveStraight():
    global leftRightSpeed;
    leftRightSpeed = 0;
    printSpeeds();
    sendDatatoRaspi();

##################################################################################
################# Variables amd functions for Robotic Arm ###################
##################################################################################
# 1. Base Motor
def incBaseMotorSpeed():
    global baseMotor
    global deltaIncrement
    baseMotor = baseMotor + deltaIncrement;
    if(baseMotor > 100):
        baseMotor = 100;
    printSpeeds()
    sendDatatoRaspi()

def decBaseMotorSpeed():
    global baseMotor
    global deltaIncrement
    baseMotor = baseMotor - deltaIncrement;
    if (baseMotor < -100):
        baseMotor = -100;
    printSpeeds()
    sendDatatoRaspi()

# 2. Base Actuator
def openBaseActuator():
    global baseActuator;
    baseActuator = 100;
    printSpeeds()
    sendDatatoRaspi()

def closeBaseActuator():
    global baseActuator;
    baseActuator = -100;
    printSpeeds()
    sendDatatoRaspi()

# 3. Arm Actuator
def openArmActuator():
    global armActuator;
    armActuator = 100;
    printSpeeds()
    sendDatatoRaspi()

def closeArmActuator():
    global armActuator;
    armActuator = -100;
    printSpeeds()
    sendDatatoRaspi()

# 4. Pitch Motor
def clockwisePitch():
    global clawPitch;
    clawPitch = clawPitch + deltaIncrement;
    printSpeeds()
    if(clawPitch > 100):
        clawPitch = 100;
    printSpeeds()
    sendDatatoRaspi()

def antiClockwisePitch():
    global clawPitch;
    clawPitch = clawPitch - deltaIncrement;
    if (clawPitch < -100):
        clawPitch = -100;
    printSpeeds()
    sendDatatoRaspi()

#5. Roll Motor
def incRollMotorSpeed():
    global clawRoll
    global deltaIncrement
    clawRoll = clawRoll + deltaIncrement;
    if(clawRoll > 100):
        clawRoll = 100;
    printSpeeds()
    sendDatatoRaspi()

def decRollMotorSpeed():
    global clawRoll
    global deltaIncrement
    clawRoll = clawRoll - deltaIncrement;
    if (clawRoll < -100):
        clawRoll = -100;
    printSpeeds()
    sendDatatoRaspi()

#6. CLAW Open close
def openCLaw():
    global clawOpenClose;
    clawOpenClose = 100;
    printSpeeds()
    sendDatatoRaspi()

def closeClaw():
    global clawOpenClose;
    clawOpenClose = -100;
    printSpeeds()
    sendDatatoRaspi()
##################################################################################
############################### print statements ##################################
##################################################################################
def printSpeeds():
    global forwardBackwardSpeed, leftRightSpeed;
    global baseMotor, baseActuator, armActuator, clawRoll, clawPitch, clawOpenClose;
    global mode
    if(mode == 0):
        stringData = str(mode) + ',' + str(forwardBackwardSpeed) + ',' + str(leftRightSpeed)
        print("MODE 0 DATA :",stringData);
    elif(mode == 1):
        stringData = str(mode) + ',' + str(baseMotor) + ',' + str(baseActuator) + ',' + str(armActuator) + ',' + str(clawPitch) + ',' + str(clawRoll)  + ',' + str(clawOpenClose);
        print("MODE 1 DATA :",stringData);

##################################################################################
############################### keyboard listener commands  ######################
##################################################################################
def on_press(key):
    global deltaIncrement
    global mode

    keyData = str(key)

    commands.append(keyData)
    commands_arr = np.asarray(commands);
    np.save('commands_arr.npy', commands_arr)

    print('pressed val = ',keyData)
    print('length', len(keyData))
    #data = '{0}'.format(key) This will work too than str(key)
    #print('data',data)
    #print('length of data',len(data))
    if(format(key)=='Key.up'):
        increaseForwardBackwardSpeed();
    elif(format(key)=='Key.down'):
        decreaseForwardBackwardSpeed();
    elif(format(key)=='Key.left'):
        decreaseleftRightSpeed();
    elif(format(key)=='Key.right'):
        increaseleftRightSpeed();
    elif(format(key) == 'Key.space'):
        stopAllMotors();
    elif(format(key) == 'Key.enter'):
        mode = mode ^ 1;             #Change Mode value
        print('MODE CHANGED - 0: Propulsion, 1: Robotic Arm')
    elif(format(key) == 'Key.shift_r'):
        moveStraight();
    else: #It is a character (length = 10) or a number (length = 3)
        if(len(keyData) == 3): # If it is a number
            shotenKeyData = keyData[1];

            if(shotenKeyData == '1'):
                deltaIncrement = 1;
            elif(shotenKeyData == '2'):
                deltaIncrement = 5;
            elif(shotenKeyData == '3'):
                deltaIncrement = 10;

            elif(shotenKeyData == 'w'): #w
                print('It is w');
                openBaseActuator();
            elif(shotenKeyData == 'd'):#d
                print('It is d');
                incBaseMotorSpeed();
            elif (shotenKeyData == 'a'):  # a
                print('It is a');
                decBaseMotorSpeed()
            elif (shotenKeyData == 's'):  # s
                print('It is s');
                closeBaseActuator();
            elif (shotenKeyData == 'r'):  # r
                print('It is r');
                decRollMotorSpeed();
            elif (shotenKeyData == 't'):  # t
                print('It is t');
                incRollMotorSpeed();
            elif (shotenKeyData == 'o'):  # o
                print('It is o');
                antiClockwisePitch();
            elif (shotenKeyData == 'p'):  # p
                print('It is p');
                clockwisePitch();
            elif (shotenKeyData == 'c'):  # c
                print('It is c');
                openCLaw();
            elif (shotenKeyData == 'v'):  # v
                print('It is v');
                closeClaw();
            elif (shotenKeyData == 'z'):  # z
                print('It is z');
                closeArmActuator();
            elif (shotenKeyData == 'x'):  # x
                print('It is x');
                openArmActuator();

            elif (shotenKeyData == 'b'):  # b
                print('It is b');
                applyBrakes();

def on_release(key):
    #print("Stop")
    if key == keyboard.Key.esc:      # Stop listener
        return False


with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()
##################################################################################
##################################################################################
