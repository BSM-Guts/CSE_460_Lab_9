# This part is copied from the provided template code
import numpy as np
import cv2 as cv
import time
from picamera2 import Picamera2 
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()
from Motor import *            
PWM = Motor()
    
Trigger = 0
COUNTER = 0
# Start the Main loop
try:  
    while True:
        # This two lines of code will be used to capture camera vision
        # Copies from template code
        frame = picam2.capture_array()
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # lower and upper boundaries of Red color in HSV color space
        lower_red_boundary = np.array([160, 30, 30])
        upper_red_boundary = np.array([180, 255, 255])
        # https://www.geeksforgeeks.org/filter-color-with-opencv/
        # This code comes from the above websites. The usage is color mask
        mask = cv.inRange(hsv, lower_red_boundary, upper_red_boundary)
        # The black region in the mask has the value of 0,
        # so when multiplied with original image removes all non-blue regions
        result = cv.bitwise_and(frame, frame, mask=mask)
        # The following code is a part of the hough transform
        # https://docs.opencv.org/3.4/d4/d70/tutorial_hough_circle.html
        gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
        gray = cv.medianBlur(gray, 5)
        rows = gray.shape[0]
        # The parameters in this function is chosen after trails and fails
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 8, param1=210, param2=20, minRadius=1, maxRadius=0)
        # define the center and radius of the circles to avoid errors
        center = 0
        radius = 0
        # The code of this loop come from the same websites
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv.circle(frame, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv.circle(frame, center, radius, (255, 0, 255), 3)
        # Break if loop
        # Comes from websites
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        # The following code will define the control policy
        # The parameters for motors and sleep function are determined after trails and fails
        if center == 0 and Trigger == 0:
            PWM.setMotorModel(0, 0, 0, 0)
            time.sleep(0.06)
            PWM.setMotorModel(-1200, -1200, 1600, 1600)
            time.sleep(0.02)
        elif center == 0 and Trigger == 1:
            time.sleep(0.05)
            COUNTER += 1
            if COUNTER >= 5:
                Trigger = 0
        else:
            Trigger = 1
            COUNTER = 0
            Base_Speed = 1000
            print("Keep Tracking")
            Difference = center[0]-300
            # The speed control will keep using the idea we implemented in previous lab
            # the speed parameter passed to the motors will be like: base speed + gain*distance
            # U is a variable used to set up the motor speed.
            # u[1], u[2], u[3], u[4] will be passed to each motor to make it work
            # use variable 'i' can decide which element in list u is passed to which motor
            if Difference < 0:
                K_gain = Difference*1.3
                u_motor = [Base_Speed+8*K_gain, Base_Speed+8*K_gain, Base_Speed-6*K_gain, Base_Speed-6*K_gain]
            elif Difference > 0:
                K_gain = -Difference*1.3
                u_motor = [Base_Speed-6*K_gain, Base_Speed-6*K_gain, Base_Speed+8*K_gain, Base_Speed+8*K_gain]
            else:
                PWM.setMotorModel(0, 0, 0, 0)
                time.sleep(0.06)
            for i in range(4):
                if u_motor[i] > 2*Base_Speed:
                    u_motor[i] = 2*Base_Speed
                elif u_motor[i] < -1.5*Base_Speed:
                    u_motor[i] = -1.5*Base_Speed
                u_motor[i] = int(u_motor[i])
            if radius < 30:
                PWM.setMotorModel(u_motor[0], u_motor[1], u_motor[2], u_motor[3])
                time.sleep(0.1)
            else:
                PWM.setMotorModel(u_motor[0], u_motor[1], u_motor[2], u_motor[3])
                time.sleep(0.06)
            # Stop all the motors at the end of loop to avoid bugs
            PWM.setMotorModel(0, 0, 0, 0)
            time.sleep(0.05)
# Close all the motors and cv2 windows when there is keyboard interrupt
except KeyboardInterrupt:
    PWM.setMotorModel(-0, -0, 0, 0)
    cv.destroyAllWindows()
# Close all the motors and cv2 windows after execution
PWM.setMotorModel(-0, -0, 0, 0)
cv.destroyAllWindows()
print("Close!")
