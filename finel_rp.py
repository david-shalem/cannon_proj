import cv2
import numpy as np
from imutils.video import VideoStream
import imutils
import time
import math
import serial

# 'conecting' the computer cam 
arduino_connection = serial.Serial(port="COM8", baudrate=9600, timeout=0.5)
vs = VideoStream(src=0).start()
time.sleep(2.0)

''' movment detection globals '''
firstFrame = None

delay_counter = 0 
FRAME_DELAY = 30 
MOVMENT_SENSITIVITY = 50
''' ------------------------- '''


''' distance detection globals '''
TENIS_RADIUS = 0.03262676333 # in meters 
FOCAL_VALUE = 900.79 # pixels
''' -------------------------- '''

''' angle of servos calculation globals '''
ANGLES_RATIO = 2.3 # the amount of angle movment in the gun per the angle movment of the horizontal servo 
CANNON_HEIGHT = 0
''' ----------------------------------- '''

def getDistance(image):
	for i in range(1):
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		lower_blue = np.array([100,150,0])
		upper_blue = np.array([140,255,255])

		# Here we are defining range of bluecolor in HSV
		# This creates a mask of blue coloured 
		# objects found in the frame.
		mask = cv2.inRange(hsv, lower_blue, upper_blue)

		# Threshold the image to get binary values
		_, thresh = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)

		contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		c = max(contours, key=cv2.contourArea)
		(x, y, w, h) = cv2.boundingRect(c)

		# Calculate the distance
		distance = ((TENIS_RADIUS * 2) * FOCAL_VALUE) / w

		key = cv2.waitKey(1) & 0xFF
		if key == ord("q"):
			break
                
		return distance

def getBlueThresh(curr_frame):
    frame = curr_frame 

    # Converts images from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100,150,0])
    upper_blue = np.array([140,255,255])
    
    # create mask that take only the pixels within the range of lower_blue -> upper_blue
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # getting a threshhold of the frame and the mask
    _, thresh = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
    return thresh


def detectBall(thresh1, thresh2):
    # get all the pixels that are white in both threshholds
    combined = cv2.bitwise_and(thresh1, thresh2)
    
    # get all white pixels
    white_pixels = np.argwhere(combined == 255) 

    if len(white_pixels) == 0:
        print("No white pixels in image")
        return (0,0,combined)
    else:
        # Find the average x and y of all white pixels
        avg_x = np.mean(white_pixels[:, 1])
        avg_y = np.mean(white_pixels[:, 0])

        return (avg_x, avg_y, combined)


def getMovementThresh(curr_frame):
    global firstFrame
    global delay_counter
    frame = curr_frame

    # resize the frame and converting him for frame with very basic details 
    # blurring the frame by decreasing and increasing his size 
    frame = imutils.resize(frame, width=500)
    (height, width) = curr_frame.shape[:2]
    frame = cv2.resize(frame, (width, height))

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)

    # taking care of the first time in the loop when the frame that we are comparing our current frame to is still empty
    if firstFrame is None:
        firstFrame = gray
        return None


    # find all the differences between the current basic frame and the last basic frame
    frameDelta = cv2.absdiff(firstFrame, gray)
    if delay_counter == FRAME_DELAY:
        firstFrame = gray
        delay_counter = 0
    else:
        delay_counter += 1

    # colecting all the big changes that were found in the frame while ignoring small changes (so stuff like shadows and
    #  small camera movment won't count as movment)   
    thresh = cv2.threshold(frameDelta, MOVMENT_SENSITIVITY, 255, cv2.THRESH_BINARY)[1]

    # dilating the resultes
    thresh = cv2.dilate(thresh, None, iterations=2)
    return thresh

def getServosAngles(x: int, distance: float):
    horizontal_angle = 0
    vertical_angle = 0

    is_negative = x < 0
    x = abs(x)

    # transfer pixels to 
    distance_x = distance,0
    x = (distance_x * x) / FOCAL_VALUE

    h = math.sqrt((distance*distance) - (x*x))
    horizontal_angle = math.atan(x/h)
    horizontal_angle /= ANGLES_RATIO
    if is_negative:
        x += 90 

    tmp = math.atan(abs(CANNON_HEIGHT)/distance)
    vertical_angle = 90 - tmp 

    return horizontal_angle, vertical_angle
     
def main():
    while True:
        # geting a frame from the computer cam
        curr_frame = vs.read()

        # getting the identifing threshes 
        thresh1 = getBlueThresh(curr_frame)
        thresh2 = getMovementThresh(curr_frame) 

        try:
            (x,y,thresh) = detectBall(thresh1, thresh2) # find the ball cordinations 

            # help prints 
            print(x,y,"===")
            cv2.circle(curr_frame, (int(x),int(y)), 10, (0,0,255), -1)
        except:
            print("continue---")
            continue
        
        # help prints
        cv2.imshow('Thresh View',thresh)
        cv2.imshow('Frame View',curr_frame)


        ball_dis = getDistance(curr_frame) # find the distance from the ball 
        (height, width) = curr_frame.shape[:2]
        x = (width / 2) - x 
        servo1, servo2 = getServosAngles(x,ball_dis) # get the servo angles required to point the cannon at the target  

        arduino_connection.write(bytes((str(servo1) + "," + str(servo2)), "utf-8")) # writing the result to the arduino 

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    vs.stop()
    arduino_connection.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()