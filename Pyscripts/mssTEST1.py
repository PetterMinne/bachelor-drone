import time

import cv2
import mss
import numpy as np
import argparse

with mss.mss() as sct:
    # Part of the screen to capture
    monitor = {'top': 40, 'left': 200, 'width': 800, 'height': 640}

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
        #cv2.THRESH_BINARY,11,3.5)
        gray = cv2.erode(gray,None,iterations = 2)
        gray =cv2.Canny(gray,100,200)
        #kernel = np.ones((2,2),np.uint8)
        #
        # gray = dilation

        # get the size of the final image
        # img_size = gray.shape
        # print img_size
        
        # detect circles in the image
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 2, 2000, param1=50, param2=18, minRadius=25, maxRadius=30)
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
                #time.sleep(0.5)
                
            # Display the picture
            cv2.imshow(' gray',gray)
            cv2.imshow('OpenCV/Numpy normal', img)

            # Display the picture in grayscale
            #cv2.imshow('OpenCV/Numpy grayscale', cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY))

            print('fps: {0}'.format(1 / (time.time()-last_time)))

            # Press "q" to quit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
