import numpy as np
from PIL import ImageGrab
import cv2
import time

from collections import deque
import argparse
import imutils


def screen_record(): 


    # define the lower and upper boundaries of the "green"
    # ball in the HSV color space, then initialize the
    # list of tracked points
    greenLower = (0, 70, 0)
    greenUpper = (10, 255, 255)
    
  
    last_time = time.time()



    while(True):
        # 800x600 windowed mode for GTA 5, at the top left position of your main screen.
        # 40 px accounts for title bar.
        printscreen =  np.array(ImageGrab.grab(bbox=(0,40,1270,720)))
        print('loop took {} seconds'.format(time.time()-last_time))
        last_time = time.time()
     
        frame= cv2.cvtColor(printscreen, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

       

        # show the frame to our screen
        cv2.imshow("Frame", frame)
        cv2.imshow('mask', mask)



        #viser img caputred
        #cv2.imshow('window',cv2.cvtColor(printscreen, cv2.COLOR_BGR2RGB))


        if cv2.waitKey(25) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

screen_record()
