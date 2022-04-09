#Adapted from GitHub - samhuff98-evo_biomech.url

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import subprocess
import imutils
from imutils.video import FPS
from imutils.video.pivideostream import PiVideoStream
import stepper as st

#setting up raspberry pi camera module
camera=PiCamera()
camera.resolution = (320,240)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320,240))
stream = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)
camera.close()
vs=PiVideoStream().start()
time.sleep(2.0)
runs = 0
count = 0

#Set up motor pins [stepPin,directionPin]
stepper1 = st.stepper([4,3])
stepper2 = st.stepper([6,5])


while True:
    x_value = 0
    y_value = 0

    frame = vs.read()
    #creating blank array for interface
    ndarray = np.full((900,1440,3), 20, dtype=np.uint8)
    grayscaled = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    kernel = np.ones((5,5), np.uint8)
    #masking out the background
    if runs < 20 or GPIO.input(GPIO_calibrate)==False:
        ret1, mask1 = cv2.threshold(grayscaled, 120, 255, cv2.THRESH_BINARY_INV)
        mask1 = cv2.dilate(mask1, kernel, iterations=1)
        runs+=1

    #insect detection mask
    cut_frame = cv2.bitwise_or(grayscaled, mask1)
    ret2, mask2 = cv2.threshold(cut_frame, 100 ,255, cv2.THRESH_BINARY)
    mask2 = cv2.bitwise_not(mask2)

    #mask2 = cv2.dilate(mask2, kernel, iterations=1)
    mask1_disp = cv2.merge([mask1,mask1,mask1])
    mask2_disp = cv2.merge([mask2,mask2,mask2])

    #tracking mode detection
    cv2.putText(ndarray, 'TRACKING MODE', (1030, 100), cv2.FONT_HERSHEY_DUPLEX, 1.5, (136, 255, 12), 2)
    cnts = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:10]
    minimum_area = 10
    for c in cnts:
        area = cv2.contourArea(c)
        if area > minimum_area:
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cX, cY), 6, (136,255,12), 2)
            x_value = cX
            y_value = cY
            cv2.putText(ndarray, 'X value: {}'.format(x), (50,805), cv2.FONT_HERSHEY_SIMPLEX, 1, (136, 255, 12), 2)
            cv2.putText(ndarray, 'Y value: {}'.format(y), (50,855), cv2.FONT_HERSHEY_SIMPLEX, 1, (136, 255, 12), 2)
            break

    frame = imutils.resize(frame, width=1000)
    ndarray[0:750,0:1000] = frame
    ndarray[190:430,1060:1380] = mask1_disp
    ndarray[470:710,1060:1380] = mask2_disp
    #text display on ndarray
    belt_speed = int(speed/170000)
    cv2.putText(ndarray, 'Belt Speed: {} mm/s'.format(belt_speed), (330, 830), cv2.FONT_HERSHEY_SIMPLEX, 1, (96, 55, 212), 2)
    cv2.putText(ndarray, '@evo_biomech', (1190, 870), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 1.2, (255, 255, 255), 2)
    cv2.namedWindow("Frame", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Frame", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow("Frame", ndarray)
    #saving video stream
    out.write(ndarray)

    if count % 10 == 0:
        if 0 < x_value < 150:
            steps = 400*(150 - x_value)
            dir = "right"
        elif 150 < x_value < 300:
            steps = 400*(x_value - 150)
            dir = "left"
        else:
            steps = 0
            dir = "right"

        stepper1.step(steps, dir, 1)

        if 0 < y_value < 150:
            steps = 400*(150 - y_value)
            dir = "right"
        elif 150 < y_value < 300:
            steps = 400*(y_value - 150)
            dir = "left"
        else:
            steps = 0
            dir = "right"

        stepper2.step(steps, dir, 1)

    count +=1

    #emergency exit
    if cv2.waitKey(1) & 0xFF == ord('c'):
        break

cv2.destroyAllWindows()
GPIO.cleanup()
ticcmd('--enter-safe-start')
ticcmd('--deenergize')
vs.stop()
