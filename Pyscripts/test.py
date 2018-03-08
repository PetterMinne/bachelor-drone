from AirSimClient import *
import numpy as np
import cv2
import os
import io
import tempfile


# for car use CarClient() MultirotorClient()
client = MultirotorClient()
client.enableApiControl(True)
#AirSimClientBase.wait_key('Press any key to take images')
# get camera images from the car

responses = client.simGetImages([ImageRequest(0, AirSimImageType.Scene, False, False)])
response = responses[0]

# get numpy array
img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 

# reshape array to 4 channel image array H X W X 4
img_rgba = img1d.reshape(response.height, response.width, 4)  

# original image is fliped vertically
#img_rgba = np.flipud(img_rgba)

cv2.imwrite('color_img.jpg', img_rgba)
cv2.imshow("image", img_rgba);
cv2.waitKey();

# write to png 
#png_image = client.write_png(os.path.normpath('test.greener.png'), img_rgba)
#img = cv2.imshow('stuff',png_image)

#print(os.path.normpath('test.greener.png'))

client.enableApiControl(False)