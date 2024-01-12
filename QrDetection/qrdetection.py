import cv2
import numpy as np
import math
import pyzbar.pyzbar as pyzbar

imcap = cv2.VideoCapture(0)
imcap.set(3, 640) # set width as 640
imcap.set(4, 480) # set height as 480
KNOWN_DISTANCE = 10
KNOWN_WIDTH = 10
distance=0
x_distance=0
y_distance=0
while True:
    success, img = imcap.read()
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    qrcodes = pyzbar.decode(img)
    for qrcode in qrcodes:
        (x, y, w, h) = qrcode.rect
        img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 3)
        focalLength = 500
        distance = (KNOWN_WIDTH * focalLength) / w
        x_distance=x*0.026458
        y_distance=y*0.026458
    cv2.imshow('face_detect', img)
    print(distance,x_distance,y_distance)
    # cv2.putText(img, distance, (50, 50),'FONT_HERSHEY_*')
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break
imcap.release()
cv2.destroyWindow('face_detect')