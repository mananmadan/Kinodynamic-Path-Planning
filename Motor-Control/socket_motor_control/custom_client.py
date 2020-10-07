import socket

s = socket.socket()
s.connect((socket.gethostname(),9999))
print("connected")
msg = s.recv(1024)
print(msg.decode("utf-8"))
