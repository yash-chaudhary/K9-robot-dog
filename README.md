# K9 Robot Dog

The COVID-19 pandemic gripped the world with its harsh lockdowns, lonely isolation, and imaginary
human contact. It turned loving families into raging chihuahua’s. But it was our pets that continued to
love us unconditionally. This report will demonstrate the development of K9 - man’s best robot friend

![Image of Physical Robot 1](https://github.com/yash-chaudhary/K9-robot-dog/blob/main/Assets/pic_1.jpeg)
![Image of Physical Robot 2](https://github.com/yash-chaudhary/K9-robot-dog/blob/main/Assets/pic_2.jpeg)

### Software Design

Python was used to program a Raspberry Pi 4 Model B to leverage the open source real-time computer vision library, openCV. Using a
camera module, this python library is able to define the landmarks on a hand(s) to figure out how many figures a person is holding up
in real-time. Once it is able to recognise the gesture it sends this number as a command via the UART communication protocol to an
Arduino Uno which reads the command and executes the routine corresponding to the command. The Arduino Uno is a microcontroller as opposed
to a microprocesser which allows to it control it's I/O peripherals quite quickly which is incredibly important for the controlled motion
of quadrupedal motion.

As an aside, a couple of hours before the reveal of the robot, the gesture recogition software had an undetected bug. To restore
motion to the robot, an ultrasonic sensor was used, where instead of using a camera module with gesture recognition the ultrasonic sensor would detect the distance between itself and an obstacle in its field of view. Different distance thresholds away from the ultrasonic sensor were defined to control the movement of the robot with a 1 second delay between each command.

![Image of Software Design](https://github.com/yash-chaudhary/K9-robot-dog/blob/main/Assets/software_design.jpeg)

### Electrical Design

The first electronics track is the power transmission used to command the servo motors and it involves a 7.4V two-cell lithium polymer battery connected to a DPDT switch. The switch controls the power supply to a buck converter (DC-DC converter) that drops the 7.4V from the LiPo batteries to a 5V supply. The output from the buck converter is a regulated 5V supply from a USB-A connector that is connected to a Raspberry Pi Model 4 B via USB-C. The USB-A port from the Raspberry Pi is then connected to the serial port of the Arduino. Then using the 5V output pin on the Arduino, the PCA9685 Servo Driver is powered by connecting the 5V output to the PCA9685 VCC pin. This VCC pin is strictly used to power the PCA9685 chip. Other pins are required to ensure the servo driver works. The ground pin of the servo driver is connected to the ground pin on the Arduino and the SDA (serial data) and SCL (serial clock) are connected to the SDA and SCL pins on newer Arduinos or analogue pin 4 and 5 respectively on older Arduinos. These two pins allow the Arduino and PCA9685 to communicate to each other via the I2C communication protocol. This completes the power supply to the Raspberry Pi, Arduino and PCA9685 that are used to command the servo motors. The second electronics track is used to power the servo motors. Again a 7.4V two cell lithium polymer battery is used and it is connected to a universal battery eliminator circuit (essentially another buck converter) that steps down the battery supply voltage from 7.4V to 6V at a maximum supply current of 10A. The output of the universal battery eliminator circuit is connected to the terminal block on the PCA9685, and it also has a SPST switch to control the power supply. Each of the 3-pin servo motors that need to be controlled are connected to the PCA9685 where the top pin is the PWM signal pin, the middle pin is the V+ supply coming from the external battery power supply, and the last pin being ground. The completes the circuit to supply power to all the servos.

![Image of Electrical Design](https://github.com/yash-chaudhary/K9-robot-dog/blob/main/Assets/circuit_diagram.png)

### Mechanical Design

Calculations were conducted find out the max loading the servo motors would withstand. After this, the parts were designed in
Solidworks with load bearing components undergoing Finite Element Analysis (FEA) to ensure they don't fail under load. These components
were the ones that were going to be 3D printed. The rest of the components were laser cut with MDF and acrylic. One of the highlights
for the design was the use of captive nut and bolt t-solts that removed the need for any permanent joints in the design. Another insight was
that traditional robot dog designs have the mass of the servo motors controlling the lower leg was overhanging from the chassis, this design can lead to instabilities due to the potential for moments to occur which can topple the robot in the roll axis. So the goal was
to find a different design that kept the distribution of mass closer to the centre of mass to increase stability and handling of the robot during movements. A servo configuration that retained both the servos within the chassis with no large mass overhangs was found. The result is that stability was increased relative to other quadruped designs as the mass is concentrated closer to the centre of mass instead of hanging off the outer parts of the legs which can cause unwanted moments when moving the robot. Also the robot deliberating had a wider
body to further increase stability

![Image of CAD design](https://github.com/yash-chaudhary/K9-robot-dog/blob/main/Assets/final_cad_model.png)
![Image of CAD exploded view](https://github.com/yash-chaudhary/K9-robot-dog/blob/main/Assets/final_cad_exploded.png)
