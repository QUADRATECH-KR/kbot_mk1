#!/usr/bin/python3
import socket
import datetime

IP   = '192.168.43.83'
PORT = 1234 

data_ = bytes(str(0), 'utf-8')

def send_ix():
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


#data_ = "TEST @ TIME : " + str(datetime.datetime.now()) + "\n"
#data_ = bytes(data_, 'utf-8')

#send_ix(data_)
