
'''
    The following code is used to send gesture commands a rpi and pi camera to an
    Arduino to control servo movements of the robot dog K9. 
'''

import serial  # module required for serial communication
import time    # module required to use delays

# imports for gesture recognition using a pi camera
import cv2
from collections import Counter
from module import findnameoflandmark,findpostion
import math

# constants
PORT = '/dev/ttyUSB0'    # serial port that Arduino is connected to
BAUD_RATE = 9600         # bits per second data transfer


on_count = 0

# --------------------------------------------------------- DRIVING CODE ---------------------------------------------------------

if __name__ == '__main__':
    ser = serial.Serial(PORT, BAUD_RATE, timeout = 1)    # initialise ser object
    ser.flush()                                          # remove any old data that was still in buffer
    
    
    # use CV2 Functionality to create a Video stream and add some values + variables
    cap = cv2.VideoCapture(0)
    tip=[8,12,16,20]
    tipname=[8,12,16,20]
    fingers=[]
    finger=[]

    # loop for continual serial communication
    while True:
        
        # --------------------------------------- CAMERA CODE --------------------------------------------
        ret, frame = cap.read() 
      
     
        # determines the frame size, 640 x 480 offers a nice balance between speed and accurate identification
        frame1 = cv2.resize(frame, (640, 480))
    
        # determine location of the joints of the fingers 
        a=findpostion(frame1)
        b=findnameoflandmark(frame1)
     
        #determine if a finger is up or down and
        if len(b and a)!=0:
            finger=[]
            if a[0][1:] < a[4][1:]: 
                finger.append(1)
                print (b[4])
          
            else:
                finger.append(0)   
        
            fingers=[] 
            for id in range(0,4):
                if a[tip[id]][2:] < a[tip[id]-2][2:]:
                    print(b[tipname[id]])

                    fingers.append(1)
    
                else:
                    fingers.append(0)
                          
        x=fingers + finger
        c=Counter(x)
        up=c[1]
        down=c[0]
        print('This many fingers are up - ', up)

        
        # -----------------------------------------------------------------------------------------------
        command = ""
        if(up == 1):
            command = "laydown\n"
        elif(up == 2):
            command = "standup\n"
        elif (up == 3):
            command = "sit\n"
        elif (up == 4):
            command = "shake\n"
        elif (up == 5):
            command = "dance\n"
        else:
            command = "standup\n"
            
        if(on_count == 0):
            time.sleep(5)
            on_count += 1
        else:
            time.sleep(1)
        
        ser.write(command.encode('utf-8'))
        status = int(ser.readline().decode('utf-8').rstrip())   # integer 1 is returned
        if (status == 1):
            print("Arduino LED turned " + command)
        else:
            print("command FAILED")
  
    


            
            

        
                
           
            
            
        