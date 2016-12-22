#!/usr/bin/python

import socket

if __name__ == "__main__":
    # initialize
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("", 5555))
    sock.listen(1)
    
    print "Starting server at {}:{}".format("127.0.0.1", 5550)

    # wait client's connection
    while True:
        csock, caddr = sock.accept()
        
        print "Connected by {}".format(caddr)

        while True:
            data = csock.recv(512)
            if not data: break
            
            print "Received data: {}".format(data)

        csock.close()

    
