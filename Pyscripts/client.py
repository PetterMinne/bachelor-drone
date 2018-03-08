import socket
 
def Main():
        host = '127.0.0.1'
        port = 5000
         
        mySocket = socket.socket()
        mySocket.connect((host,port))
        x=0
        y=7
         
        messagex = str(x) +'#'+str(y)
        
        while x != 10:
                mySocket.send(messagex.encode())
                
                data = mySocket.recv(1024).decode()

                            
                 
                print ('Received from server: ' + data)
                x = x+1
                messagex = str(x) + '#'+str(y)
                 
        mySocket.close()
 
if __name__ == '__main__':
    Main()
