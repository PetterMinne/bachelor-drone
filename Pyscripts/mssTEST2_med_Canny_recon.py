import time
import socket
import cv2
import mss
import numpy as np
import argparse
host = '127.0.0.1'
port = 5000

with mss.mss() as sct:
    # Part of the screen to capture
    monitor = {'top': 40, 'left': 0, 'width': 800, 'height': 600}
    #text =np.zeros([150,300])
    
         
    mySocket = socket.socket()
    mySocket.connect((host,port))
    #time.clock()
    timer =0
    
    while 'Screen capturing':
        last_time = time.time()
        
        # Get raw pixels from the screen, save it to a Numpy array
        img = np.array(sct.grab(monitor))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


        # apply GuassianBlur to reduce noise. medianBlur is also added for smoothening, reducing noise.
        #gray = cv2.GaussianBlur(gray,(3,3),0);
        #gray = cv2.medianBlur(gray,5)
    
        # Adaptive Guassian Threshold is to detect sharp edges in the Image. For more information Google it.
        #gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
        #    cv2.THRESH_BINARY,11,3.5)
        
        #gray = np.logical_and(gray,text)
        # gray = erosion
   
        #gray = cv2.dilate(gray,kernel,iterations = 1)


        gray = cv2.erode(gray,None,iterations = 3)
        gray =cv2.Canny(gray,100,120)
        #kernel = np.ones((1,1),np.uint8)
        ##fjerner status text fra simulering.
        #gray[80:230,0:300] =text
        # gray = dilation

        # get the size of the final image
        # img_size = gray.shape
        # print img_size
        
        # detect circles in the image
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 2, 2000, param1=1, param2=70, minRadius=0, maxRadius=30)
        # print circles
        
        # ensure at least some circles were found
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
           
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                # draw the circle in the output image, then draw a rectangle in the image
                # corresponding to the center of the circle
                cv2.circle(img, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(img, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                #cv2.circle(gray, (x, y), r, (0, 255, 0), 4)
                #cv2.rectangle(gray, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                #time.sleep(0.5)
            
            #Sender kordinat for sirkel til server en gang hvert sekund.
            if time.time() > (timer+1):
                message= str(x)+ "#"+ str(y)
                mySocket.send(message.encode())   
                timer = time.time()
            



            
            # Display the picture
        cv2.imshow('OpenCV/Numpy normal', img)

            # Display the picture in grayscale
        cv2.imshow('OpenCV/Numpy grayscale',gray)
            #            cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY))

        print('fps: {0}'.format(1 / (time.time()-last_time)))

            # Press "q" to quit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            mySocket.close()
            cv2.destroyAllWindows()
            break
