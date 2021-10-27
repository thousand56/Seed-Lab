import cv2 as cv
import numpy as np
from picamera import PiCamera
import smbus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd


#----------- LCD Control ----------------

lcd_columns = 16
lcd_rows = 2

#init i2c bus
i2c = board.I2C()

#init lcd class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c,lcd_columns,lcd_rows)

lcd.clear()
lcd.color = [50,0,50]

# ----------- Bus communication ------------

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(value):
  bus.write_byte(address, value)
#  bus.write_byte_data(address, 0, value)
  return -1

def readNumber():
  number = bus.read_byte(address)
#  number = bus.read_byte_data(address, 1)
  return number

#while True:
 # var = input("Enter 1 – 9: ")
  #if not var:
 #   continue
#  break

#writeNumber(var)
#print ("RPI: Hi Arduino, I sent you ", var)
# sleep one second
#time.sleep(1)

#number = readNumber()
#print ("Arduino: Hey RPI, I received a digit ", number)



#------------ Computer Vision ---------------
#calculates the given position for an image, with the thresholded frame as the input
def findPhi(thresh):
    #some constants 
    hexInd = np.nonzero(thresh)
    #print (hexInd[0])
    try:        
        meanInd = np.mean(hexInd, axis=1)

        xVal = int(meanInd[1])
        yVal = int(meanInd[0])
        xRes = 320*2
        yRes = 240
        xFOV = 53
        yFOV = 41
    
        #calculations
        xDist = -(xVal-xRes/2)/(xRes/2) 
        xPhi = (xFOV/2)*xDist
        yDist = (yVal-yRes/2)/(yRes/2) 
        yPhi = (yFOV/2)*yDist
        
        limit_xphi = round(xPhi,2)
        lcd.message = "Angle: " + str(limit_xphi)
        print("x value", xPhi)
        print("y value", yPhi)
    
            
            
    except ValueError:
        lcd.clear()
        lcd.message = "no marker"
       # print("no marker")
       
        
#captures and processes video, sends thresholded image to findQuad
def videoProcess():
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("Can't open the camera")
        exit()
   # yellow_lower = np.array([100,150,0])
   # yellow_upper = np.array([140,255,255])
    blue_lower = np.array([100,150,0],np.uint8)
    blue_upper = np.array([140,255,255],np.uint8)
    while True:
        time.sleep(0.1)
        ret, frame = cap.read()
        if not ret:
            print("can't recieve frame, exiting")
            break
        
        #isolating yellow
   #     height, width = frame.shape[:2]
    #    res = cv.resize(frame, (width//2,height//2), interpolation = cv.INTER_CUBIC)
        hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
        hsv = cv.blur(hsv,(5,5))
        mask = cv.inRange(hsv,blue_lower,blue_upper)
        out = cv.bitwise_and(frame,frame,mask= mask)
        #morphological transformations
        kernel = np.ones((5,5),np.uint8)
     #   morph = cv.morphologyEx(out, cv.MORPH_CLOSE,kernel)
       # morph = cv.morphologyEx(morph, cv.MORPH_OPEN,kernel)
        
        #contours/thresholding
        gray = cv.cvtColor(out, cv.COLOR_BGR2GRAY)
    #cv.imshow("test", gray)
        (ret, thresh) = cv.threshold(gray, 10, 255,cv.THRESH_BINARY)
        #type(thresh)
        #edges = cv.Canny(thresh,100,200)
        cv.imshow("thresh", thresh)
        cv.imshow("raw", frame)
        findPhi(thresh)
        
        if cv.waitKey(1) == ord('q'):
            break
    cap.release()
    cv.destroyAllWindows()
    
videoProcess()