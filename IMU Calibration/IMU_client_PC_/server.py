import socket
import datetime

import numpy
from threading import Thread
from time import sleep

IP   = '192.168.43.2'
PORT = 1234 

data_ = bytes(str(0), 'utf-8')


def send_ix():
    global data_
    serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serv.bind((IP, PORT))
    serv.listen(5)
    while True:
        conn, addr = serv.accept()
        from_client = ''
        while True:
            #data_ = "TEST @ TIME : " + str(datetime.datetime.now()) + "\n"
            #data_ = bytes(data_, 'utf-8')
            try: 
                data = conn.recv(4096)
                if not data: 
                    break
    
                from_client = data.decode('utf8')
                #print(from_client)
                #tmp = bytes(("Received Data : " + from_client + "\n") ,'utf-8')
                conn.send(data_)
    
            except Exception as e:
                print(str(e))
                break
    
        conn.close()


##data_ = "TEST @ TIME : " + str(datetime.datetime.now()) + "\n"
#data_ = [numpy.array([123454, 122151, 1215481]), numpy.array([123454, 122151, 1215481]), numpy.array([123454, 122151, 1215481])]
##data_ = "TEST @ TIME : " + str(datetime.datetime.now()) + "\n"
#print(str(data_))
#data_ = bytes(str(data_), 'utf-8')




k = 0

thread_ = Thread(target = send_ix, args = ())
thread_.start()

while True:
   k += 1
   print(k)
   data_ = bytes(str(k), 'utf-8')
   sleep(0.5)

thread_.join()
