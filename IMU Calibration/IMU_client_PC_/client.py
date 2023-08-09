import socket
import datetime
from time import sleep

IP   = '192.168.43.83'
PORT = 1234

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

val_ = "0"

exit_sig = 0

update_rate = 0.001


def receive():
    global val_
    try:
        client.connect((IP, PORT))

    except Exception as e:
        pass
        #print(str(e))
        #return NULL

    try:
        #data = "TEST @ TIME : " + str(datetime.datetime.now()) + "\n"
        data = "#" + "\n"
        data = bytes(data, 'utf-8')
    
        #while True:
        client.send(data)
        from_server = client.recv(4096)
        #print (from_server.decode())
        #sleep(0.3)
        
        val_ = from_server.decode()
        sleep(0.1)

        return from_server.decode()

    except Exception as e:
        print(str(e))
        return 0


def receive_loop():
    global val_, exit_sig, client
    #client.connect((IP, PORT))
    while True:
        try:
            client.connect((IP, PORT))
            try:
                #data = "TEST @ TIME : " + str(datetime.datetime.now()) + "\n"
                data = "#" + "\n"
                data = bytes(data, 'utf-8')
                print("Connected To Server at : ", str(IP) + ":" + str(PORT))
            
                while True:
                    client.send(data)
                    from_server = client.recv(4096 * 1000)
                    #print (from_server.decode())
                    #sleep(0.3)
                    val_ = from_server.decode()

                    if(exit_sig):
                        try:
                            client.close()
                        except Exception as e:
                            print(str(e))
                        exit()
                    sleep(update_rate)
            
            except Exception as e:
                print(str(e))
                try:
                    client.close()
                except Exception as e:
                    print(str(e))
                sleep(update_rate)

        except Exception as e:
            print(str(e))
            print("Check If Server Is Running")
            sleep(0.1)
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            if(exit_sig):
                try:
                    client.close()
                except Exception as e:
                    print(str(e))
                exit()

        sleep(update_rate)


#while True:
#    val_ = receive()
#    print("VAL : ", val_)
#    sleep(0.1)
def close():
    client.close()


"""
# receive loop old
def receive_loop():
    global val_
    client.connect((IP, PORT))
    while True:
        
        #data = "TEST @ TIME : " + str(datetime.datetime.now()) + "\n"
        data = "#" + "\n"
        data = bytes(data, 'utf-8')
    
        while True:
            client.send(data)
            from_server = client.recv(4096)
            #print (from_server.decode())
            #sleep(0.3)
            
            val_ = from_server.decode()
            sleep(0.1)
"""