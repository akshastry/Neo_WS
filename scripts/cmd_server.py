#!/usr/bin/env python

#python imports
import time
import math
import traceback

#ROS imports
import rospy
from std_msgs.msg import UInt8

# Server imports
import socket
import threading

HEADER = 64
PORT = 8080
SERVER = '10.104.48.224'
ADDR = (SERVER, PORT)
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
# SOCKET_AMOUNT = 5

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(ADDR)

def handle_client(conn, addr):
    global cmd, pub
    print("[NEW CONNECTION] connected.")
    connected = True
    while connected:
        msg_length = conn.recv(HEADER).decode(FORMAT)
        if msg_length:
            msg_length = int(msg_length)
            msg = conn.recv(msg_length).decode(FORMAT)
            if msg == DISCONNECT_MESSAGE:
                print(DISCONNECT_MESSAGE)
                print("Connection has been terminated.")
                break
            else:
                # print("[{addr}] {msg}")
                if msg == "hover":
                    print("Ready to hover")
                    cmd.data = 1
                    pub.publish(cmd)
                    
                elif msg == "forward":
                    print("Forward")
                    cmd.data = 2
                    pub.publish(cmd)

                elif msg == "land":
                    print("Landing")
                    cmd.data = 3
                    pub.publish(cmd)
                        
                else:
                    print("Command not recognized")



def main():
    global pub, cmd

    rospy.init_node('gcs', anonymous=True)

    cmd = UInt8()
    cmd.data = 0
    pub = rospy.Publisher('/gcs/cmd', UInt8, queue_size=1)

    server.listen(1)
    print("Server is listening on "+ SERVER)

    while True:
        conn, addr = server.accept()
        thread = threading.Thread(target=handle_client, args=(conn, addr))
        thread.start()
        print("[ACTIVE CONNECTIONS] {threading.activeCount() - 1}")

    # rate = rospy.Rate(100)
    # while not rospy.is_shutdown():
    #     try:
    #         conn, addr = server.accept()
    #         thread = threading.Thread(target=handle_client, args=(conn, addr))
    #         thread.start()
    #         print("[ACTIVE CONNECTIONS] {threading.activeCount() - 1}")

    #     except Exception:
    #         traceback.print_exc()
    #         indent = 1
    
    #     rate.sleep()
                        


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass