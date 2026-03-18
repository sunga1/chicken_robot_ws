import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("192.168.1.200", 30002))

sock.sendall(b"MOVE_J:0,0,0,0,0,0\n")

sock.close()
