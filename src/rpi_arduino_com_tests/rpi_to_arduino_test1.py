
'''
    The following code is used to send commands to an Arduino to turn its
    in-built LED on and off. This is used to demonstrate serial communication
    using the UART protocol between a Raspberry Pi 4 and Arduino Uno
'''

import serial  # module required for serial communication
import time    # module required to use delays

# constants
PORT = '/dev/ttyUSB0'    # serial port that Arduino is connected to
BAUD_RATE = 9600         # bits per second data transfer


# driving code
if __name__ == '__main__':
    ser = serial.Serial(PORT, BAUD_RATE, timeout = 1)    # initialise ser object
    ser.flush()                                        # remove any old data that was still in buffer
    
    # loop for continual serial communication
    while True:
        command = input("on or off?\n")
        command = command + "\n"        # convert to format that Arduino is expecting
        ser.write(command.encode('utf-8'))
        status = int(ser.readline().decode('utf-8').rstrip())   # integer 1 is returned
        if (status == 1):
            print("Arduino LED turned " + command)
        else:
            print("command FAILED")
        
                
           
            
            
        