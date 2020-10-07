##### Attrey Bhatt Codes - https://github.com/attreyabhatt/Reverse-Shell ###########
# If we are hacker then this file will go to our server that has a static ip address
#importing socket so that we can connect two computer
import socket
#importing PySerial and time
import serial
import time, threading
import motor_ibt2
import motor_l298n
import os


###################ARDUINO SERIAL OBJECT#################################################
#serialPortMac = '/dev/tty.usbmodem14101'
#serialPortPi = '/dev/ttyACM0'
#arduinoSerial = serial.Serial(serialPortMac, 9600, timeout = 1)
#curently fL -> 2,3
mode = 0;
motorspeed1 = 0
motorspeed2 = 0
forward_left_motor = motor_ibt2.motor1_ibt2(2,3)
forward_right_motor = motor_ibt2.motor1_ibt2(14,15)

backward_left_motor = motor_ibt2.motor1_ibt2(4,17)
backward_right_motor = motor_ibt2.motor1_ibt2(18,23)

################################
# VARIABLES FOR ROBOTIC ARM:
#################################
baseMotorSpeed = 0
baseActuator = 0;
armActuator = 0;
clawPitch = 0;
clawRoll = 0;
clawOpenClose = 0;

Motor1_baseMotor = motor_ibt2.motor1_ibt2(25,24);             #IBT2 (2 pin)
Motor2_baseActuator = motor_l298n.motor1_L298n_NOPWM(6,13);#L298n (2 pin)
Motor3_armActuator = motor_l298n.motor1_L298n_NOPWM(19,26); #L298n (2 pin)

Motor4_clawPitch = motor_l298n.motor1_L298n(12,1,10); #L298n (2 pin)

Motor5_clawRoll = motor_l298n.motor1_L298n(21,20,16);     #L298n (3 pin)
Motor6_clawOpenClose = motor_l298n.motor1_L298n_NOPWM(7,8);#L298n (2 pin)
#
#########################################################
#########################################################
groundIP = "192.168.185.58"
def IPCheckRoutine():
    print(time.ctime())
    response = os.system("ping -c 1 " + groundIP)
    if response == 0:
        print('BASE - CONNECTED');
    else:
        if(mode == 0):
            dataFromBase = "0,0,0"
            index1 = dataFromBase.index(',')
            propulsion(dataFromBase,index1);
        else:
            dataFromBase = "1,0,0,0,0,0,0"
            index1 = dataFromBase.index(',')
            roboticArm(dataFromBase,index1);

    threading.Timer(3, IPCheckRoutine).start()
#########################################################
######################################

######################################################################################################################
########## Function to Create a Socket ( socket connect two computers)
######################################################################################################################
def create_socket():
    try:
        #Creating following 3 global variables
        global host
        global port
        global s         #This is socket variable which is named s

        #Assigning values to these 3 global variables
        host = ""
        port = 9999
        s = socket.socket()    # Creating a socket and assigning it to s

    except socket.error as msg:
        print("Socket creation error: " + str(msg))


######################################################################################################################
########## # Binding the socket and listening for connections:
# Before accepting connection we listen for connections after binding host and port with the socket
######################################################################################################################
def bind_socket():
    try:
        # Declaring them again so that we can use the above global variable
        global host
        global port
        global s
        print("Binding the Port: " + str(port))

        s.bind((host, port))
        s.listen(5)

    except socket.error as msg:
        print("Socket Binding error" + str(msg) + "\n" + "Retrying...")
        bind_socket()

######################################################################################################################
###########   Establish connection with a client (socket must be listening)
######################################################################################################################
def socket_accept():
    #s.accept retuens : conn: object of a conversation and address is a list of IP adress and a port
    conn, address = s.accept()
    print("Connection has been established! |" + " IP " + address[0] + " | Port" + str(address[1]))
    read_commands(conn) #A function defined below to send command to client
    conn.close() #whenever the connection has been establised, at the end we want to close the connection

######################################################################################################################
###########  # Send commands to client/victim or a friend
######################################################################################################################
def send_commands(conn,data):
    conn.send(str.encode(data))
######################################################################################################################
###########  # Send commands to client/victim or a friend
######################################################################################################################

def strToInt(string):
    if(len(string) == 0):
#        print('string length 0')
        return 0;
    x=0
    flag = 0
    if(string[0]=='-'):
        flag=1

    for i in range (0,len(string)):
                    if string[i].isdigit():
                        x+=int(string[i])*10**int(len(string)-i-1)
#                        print('In strToInt',i,x)
    if (flag ==1):
        return (-1)*x
    else:
        return x

def propulsion(dataFromBase, index1):
    global mode,motorspeed1, motorspeed2, forward_left_motor, forward_right_motor, backward_left_motor, backward_right_motor;

    index2 = dataFromBase.index(',',index1+1)

    motorspeed = dataFromBase[index1+1:index2]
    a = strToInt(motorspeed)
    motorspeed1 = a
    motorspeed2 = a

    #print('motorspeed1',motorspeed1)
    motorspeed = dataFromBase[index2+1:]

    print(motorspeed)
    b = strToInt(motorspeed)

    motorspeed1 -= b
    motorspeed2 += b
    if (motorspeed1 > 100):
        motorspeed1 = 100
    elif (motorspeed1 < -100):
        motorspeed1 = -100

    if (motorspeed2 > 100):
        motorspeed2 = 100
    elif (motorspeed2 < -100):
        motorspeed2 = -100

    print('motorspeed1',motorspeed1)
    print('motorspeed2',motorspeed2)

    forward_left_motor.moveMotor(motorspeed2)
    backward_left_motor.moveMotor(motorspeed2)

    forward_right_motor.moveMotor(motorspeed1)
    backward_right_motor.moveMotor(motorspeed1)

def printRoboticArmVariables():
    print(baseMotorSpeed, baseActuator, armActuator, clawPitch, clawRoll, clawOpenClose)

def roboticArm(dataFromBase, index1):
    global baseMotorSpeed, baseActuator, armActuator, clawRoll, clawPitch, clawOpenClose;
    global Motor1_baseMotor, Motor2_baseActuator, Motor3_armActuator, Motor4_clawPitch, Motor5_clawRoll, Motor6_clawOpenClose;

    index2 = dataFromBase.index(',',index1+1)
    StrbaseMotorSpeed = dataFromBase[index1+1:index2]
    baseMotorSpeed = strToInt(StrbaseMotorSpeed);
    Motor1_baseMotor.moveMotor(baseMotorSpeed);
    Motor1_baseMotor.printMotor('Motor1_baseMotor');

    index3 = dataFromBase.index(',',index2+1)
    StrbaseActuator = dataFromBase[index2+1:index3]
    baseActuator = strToInt(StrbaseActuator);
    Motor2_baseActuator.moveMotor(baseActuator);
    Motor2_baseActuator.printMotor('Motor2_baseActuator');

    index4 = dataFromBase.index(',',index3+1)
    StrarmActuator = dataFromBase[index3+1:index4]
    armActuator = strToInt(StrarmActuator);
    Motor3_armActuator.moveMotor(armActuator);
    Motor3_armActuator.printMotor('Motor3_armActuator');

    index5 = dataFromBase.index(',',index4+1)
    StrclawPitch = dataFromBase[index4+1:index5]
    clawPitch = strToInt(StrclawPitch);
    Motor4_clawPitch.moveMotor(clawPitch);
    Motor4_clawPitch.printMotor('Motor4_clawPitch');

    index6 = dataFromBase.index(',',index5+1)
    StrclawRoll = dataFromBase[index5+1:index6]
    clawRoll = strToInt(StrclawRoll);
    Motor5_clawRoll.moveMotor(clawRoll);
    Motor5_clawRoll.printMotor('Motor5_clawRoll');

    StrclawOpenClose = dataFromBase[index6+1:]
    clawOpenClose = strToInt(StrclawOpenClose);
    Motor6_clawOpenClose.moveMotor(clawOpenClose);
    Motor6_clawOpenClose.printMotor('Motor6_clawOpenClose');

    printRoboticArmVariables();


def read_commands(conn):
    global mode,motorspeed1, motorspeed2, forward_left_motor, forward_right_motor, backward_left_motor, backward_right_motor;

    IPCheckRoutine()
    while True:
        dataFromBase = str(conn.recv(1024),"utf-8")
        print("\n Received Data = "+dataFromBase)
        #        print('lengthOfData', len(dataFromBase))
        if(len(dataFromBase) > 3):
            send_commands(conn,'YES')
            index1 = dataFromBase.index(',')
            modeStr = dataFromBase[0:index1]

            mode = strToInt(modeStr)

            if(mode == 0):
                propulsion(dataFromBase,index1);
            elif(mode == 1):
                roboticArm(dataFromBase,index1);

        else:
            send_commands(conn,'NO')

######################################################################################################################
###########  # Process Data from raspberrypi to Arduino
######################################################################################################################
#def processDataToArduino(data):
 #   arduinoSerial.write(str(data).encode())

#####################################################################################################################
###########  # Remove b'' and\r\n from the string
######################################################################################################################
def makeDataWhatArduinoSent(data):
    return data[2:len(data)-5]

######################################################################################################################
###########  # MAIN
######################################################################################################################
def main():
    create_socket()
    bind_socket()
    socket_accept()
############################################################
#Sending fake data
#    processDataToArduino('1,1001,1002,1003,1004,1005,1006');
#    time.sleep(2)
#    processDataToArduino('0,0,0,0,0,0,0');
########################################

main()
