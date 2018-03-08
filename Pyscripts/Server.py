import socket
import subprocess
import cv2
import numpy as np
# Start a socket listening for connections on 0.0.0.0:8000 (0.0.0.0 means
# all interfaces)
server_socket = socket.socket()
server_socket.bind(('0.0.0.0', 8000))
server_socket.listen(0)

# Accept a single connection and make a file-like object out of it
connection = server_socket.accept()[0].makefile('rb')
try:
    # Run a viewer with an appropriate command line. Uncomment the mplayer
    # version if you would prefer to use mplayer instead of VLC
    #cmdline = ['C:\Program Files (x86)\VideoLAN\VLC\VLC ', '--demux', 'jpeg', '-']
    #cmdline = ['mplayer', '-fps', '25', '-cache', '1024', '-']
    #player = subprocess.Popen(cmdline, stdin=subprocess.PIPE)
    while True:
        # Repeatedly read 1k of data from the connection and write it to
        # the media player's stdin
        data = connection.read(-1)
        print(type(data))
        if not data:
            break

        #data = np.asarray(data)
        #cv2.imshow("cmd",data)
finally:
    connection.close()
    server_socket.close()
    #player.terminate()