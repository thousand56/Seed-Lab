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





#------------ Computer Vision ---------------
#calculates the given position for an image, with the thresholded frame as the input
def findPhi(thresh):
    #some constants
    output = [0,0]
    hexInd = np.nonzero(thresh)
    if(hexInd[0].size == 0):
        return([99,99])
    else:
    
        try:        
            meanInd = np.mean(hexInd, axis=1)
    
            xVal = int(meanInd[1])
            yVal = int(meanInd[0])
            xRes = 320*2
            yRes = 240*2
            xFOV = 53
            yFOV = 41
    
        #calculations
            xDist = -(xVal-xRes/2)/(xRes/2) 
            xPhi = (xFOV/2)*xDist
            yDist = (yVal-yRes/2)/(yRes/2) 
            yPhi = (yFOV/2)*yDist
        
       # limit_xphi = round(xPhi,2)
       # lcd.message = "Angle: " + str(limit_xphi)
            output[0] = xPhi
            output[1] = yPhi
        
       # print("x value", xPhi)
       # print("y value", yPhi)
    
            
            return output
        except RuntimeWarning:
            return [99,99]
        except ValueError:
            lcd.clear()
            lcd.message = "no marker"
            return[99,99]
       # print("no marker")
       
        
#captures and processes video, sends thresholded image to findQuad
def videoProcess():
    cap = cv.VideoCapture(0)
    i = 0
    
    if not cap.isOpened():
        print("Can't open the camera")
        exit()
        
    blue_lower = np.array([100,100,0],np.uint8)
    blue_upper = np.array([140,255,255],np.uint8)
    
    start = time.time()
    captureDur = 2
    while(int(time.time()-start) < captureDur):
        ret, frame = cap.read()
        if not ret:
            print("can't recieve frame, exiting")
            break
    #cropping
        height, width, channels = frame.shape
        croppedFrame = frame[int(height/2):height, 0:width]
    #filtering
        hsv = cv.cvtColor(croppedFrame,cv.COLOR_BGR2HSV)
        hsv = cv.blur(hsv,(5,5))
        mask = cv.inRange(hsv,blue_lower,blue_upper)
        out = cv.bitwise_and(croppedFrame,croppedFrame,mask= mask)
        
        kernel = np.ones((5,5),np.uint8)
        morph = cv.morphologyEx(croppedFrame, cv.MORPH_OPEN,kernel)

        #contours/thresholding
        gray = cv.cvtColor(out, cv.COLOR_BGR2GRAY)
        (ret, thresh) = cv.threshold(gray, 10, 255,cv.THRESH_BINARY)
                
        cv.imshow("thresh", thresh)
        cv.imshow("raw", croppedFrame)
    #    findPhi(thresh)
        
        if cv.waitKey(1) == ord('q'):
            break
        
    cap.release()
    cv.destroyAllWindows()
    return(thresh)

            #15,   16.   17   18   19   20   21   22  23     24    25   26    27    28    29  ..30 inches away
distLUT = [-0.42,-1.11,-1.6,-2.1,-2.5,-2.8,-3.2,-3.5,-3.76,-4.01,-4.1,-4.35,-4.52,-4.61,-4.78,-4.87]
def main():
    #this gets 5 seconds of frames, processes them, and stores them in an array 
    frame = videoProcess()
    allAngles = np.empty(0)
    #process each array and output the x and y angles
    angles = [0,0]
    
    angles = findPhi(frame)
    
    
    if (angles[0] == 99):
        blue = 0
    else:
        blue = 1
    ind = 0
    dist = 31
    for lookup in distLUT:
        if(angles[1] > lookup):
            dist = ind+14
            break
        ind += 1
    output = [blue, dist, int(angles[0])]
    
    print("blue found = ", blue)
    print("distance = ",dist)
    print(angles[0])
    
    

    return(output)
            

        


state = 1
#dictionary defining the possible states
state_dictionary = {
    0 : "hold",
    1 : "search",
    2 : "refineAng",
    3 : "driveForward",
    4 : "finalStretch",
    5 : "refineAng2"
    }

#hold, stays in there, ends the program
def state0(action):
    if action == 0:
        return 0

#
def state1(action):
    if action == 2:
        return 2
    elif action == 1:
        return 1


def state2(action):
    if action == 3:
        return 3

    
def state3(action):
    if action == 5:
        return 5
    elif action == 3:
        return 3
    
def state5(action):
    if action == 4:
        return 4
    elif action == 5:
        return 5

    
def state4(action):
    if action == 0:
        return 0
    else:
        return 4
    
    #use try and except to set the blue boolean value in main
    
    #operation loop, runs through the user input and moves between states to determine if the desired strings are contained
while state is not None:
    print("current state: " +state_dictionary[state])
    action = 1
    
        
    if state == 0:
        #writeNumber(0)
        action = 0
        state = state0(action)
    #search
    elif state == 1:
        writeNumber(128)
        time.sleep(2)
        [blue, dist, angle] = main()
        if(blue):
            action = 2
            state = state1(action)
    #refine    
    elif state == 2:
        [blue, dist, angle] = main()
        
        if(angle > 3):
            angle = angle + 100
            writeNumber(angle)
        elif(angle < -3):
            angle = -angle
            angle = angle + 175
            writeNumber(angle)
    
        time.sleep(2)
        if(abs(angle) < 3):
            action = 3
            state = state2(action)
                
    #drive forward
    elif state == 3:
        print("going forward")
        writeNumber(48)
        time.sleep(5)
        action = 5
        state = state3(action)
        
    #refine 2    
    elif state == 5:
        [blue, dist, angle] = main()
        
        if(angle > 4):
            angle = angle + 100
            writeNumber(angle)
        elif(angle < -4):
            angle = -angle
            angle = angle + 175
            writeNumber(angle)
    
        time.sleep(2)
        if(abs(angle) < 4):
            action = 4
            state = state5(action)
        
    #final stretch
    elif state == 4:
        [blue, dist, angle] = main()
        if(dist > 30):
            print("dist is greater than 30")
            writeNumber(10)
            time.sleep(3)
        elif(dist < 20):
            print("dist less than 20")
            writeNumber(dist)
            time.sleep(3)
            writeNumber(250)
            action = 0
            state = state4(action)
        elif(dist < 30):
            print("dist less than 30")
            dist = dist - 18
            writeNumber(dist)
            time.sleep(3)
    else:
        print("invalid state")
             
    
    
    
