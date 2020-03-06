import socket
s = socket.socket()
s.bind((socket.gethostname(),9999))
s.listen(5)

while True:
    conn, address = s.accept()
    print("Connection has been established! |" + " IP " + address[0] + " | Port" + str(address[1]))
    read_commands(conn) #A function defined below to send command to client
    conn.close()

def read_commands(conn):
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
                print("dataFromBase")
                print(dataFromBase)
                print("index1")
                print(index1)
