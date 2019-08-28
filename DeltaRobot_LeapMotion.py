#To find Inverse Kinematics Of A Delta Robot
#Reference:
#Paper Name: The Delta Parallel Robot: Kinematics Solutions; Author name: Robert L. Williams II, Ph.D, Mechanical Engineering, Ohio University; Date: October 2016
#Please download the above mentioned paper. It has been kept in https://github.com/giridharanponnuvel/Delta-Robot-Inverse-Kinematics. Thank You!
#Use Python 2.7 compiler
##################################################
#Author: Giridharan P
#Date: 16-03-2019
#Credits: Thanks to Asst.Prof Rajeev Lochana G C, Mechanical Department, Amrita School of Engineering, Bangalore
##################################################
##################################################


import warnings, serial, time, math, cmath, thread, Leap, sys, serial.tools.list_ports

#Connecting to the arduino ComPort3
ArduinoSerial = serial.Serial('COM3',9600)

# Global variable declaration:
#Set the phycical robot bicep arm per the InitialConditionTheta
InitialConditionTheta = float(45.00)

#setting current position as InitialConditionTheta
currentTheta11=float(InitialConditionTheta)
currentTheta12=float(InitialConditionTheta)
currentTheta13=float(InitialConditionTheta)

currentTheta21=float(InitialConditionTheta)
currentTheta22=float(InitialConditionTheta)
currentTheta23=float(InitialConditionTheta)
#assume the position of the end effector is (0,0,0)
x = 0
y = 0
z = 0



# Sb,Sp,L variable declaration:
try:
    Sb = float(raw_input("Base equilateral triangle side: "))
except ValueError:
    Sb = float(600)
print("Sb: %.4f" % (Sb))

try:
    Sp = float(raw_input("Platform equilateral triangle side: "))
except ValueError:
    Sp = float(100)
print("Sp: %.4f" % (Sp))

try:
    L = float(raw_input("Bicep arm length: "))
except ValueError:
    L = float(150)
print("L: %.4f" % (L))

try:
    l = float(raw_input("Fore arm length: "))
except ValueError:
    l = float(300)
print("l: %.4f" % (l))
print



#Map function - used to combine Leap Motion workspace with Delta robot workspace
def valMap(orgValue, inBegin, inEnd, outBegin, outEnd):
    return (outBegin + (outEnd - outBegin) * ((orgValue - inBegin) / (inEnd - inBegin)))

#Leap motion function
class SampleListener(Leap.Listener):

    def on_init(self, controller):
        print ("Initialized")

    def on_connect(self, controller):
        print ("Connected")

    def on_disconnect(self, controller):
        print ("Disconnected")

    def on_exit(self, controller):
        print ("Exited")

    def on_frame(self, controller):
        #import global variables as local:
        global x
        global y
        global z

        frame = controller.frame()

        pointable = frame.pointables.rightmost
        position = pointable.tip_position

        #print ("orgValue of x: %d" % (
        #    int(position.x)))

        x = int(round(valMap(position.x, -100, 100, -20, 20)))
        z = int(round(valMap(position.y, -100, 100, -20, 20)))
        y = int(round(valMap(position.z, -100, 100, -20, 20)))


        #Run the sub_main function continuesly
        sub_main()

        if not frame.hands.is_empty:
            print ("")

def main():
    listener = SampleListener()
    controller = Leap.Controller()
    controller.add_listener(listener)

    print ("Press Enter to quit...")
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)

		
		
		
		
def sub_main():
    #Declaration of global variables as local:
    global currentTheta11
    global currentTheta12
    global currentTheta13
    global currentTheta21
    global currentTheta22
    global currentTheta23
#####################################################################
    # x,y,z variable declaration:
    global x
    global y
    global z

    print "Position of the tip:"
    print("x: %.4f" % (x))
    print("y: %.4f" % (y))
    print("z: %.4f" % (z))
#####################################################################
    # Wb,Ub,Wp,Up declaration:
    Wb = float(((math.sqrt(3)) / 6) * Sb)
    #print("Wb: %.4f" % (Wb))

    Ub = float(((math.sqrt(3)) / 3) * Sb)
    # print("Ub: %d" % (Ub))

    Wp = float(((math.sqrt(3)) / 6) * Sp)
    # print("Wp: %d" % (Wp))

    Up = float(((math.sqrt(3)) / 3) * Sp)
    # print("Up: %d" % (Up))
    #####################################################################
    a = float(Wb - Up)
    # print("a: %.4f" % (a))

    b = float((Sp * 0.5) - (((math.sqrt(3)) * 0.5) * Wb))
    # print("b: %.4f" % (b))

    c = float(Wp - (0.5 * Wb))
    # print("c: %.4f" % (c))

    # print
    #####################################################################
    # Inverse Kinematics:
    E1 = float(2 * L * (y + a))
    # print("E1: %.4f" % (E1))

    F1 = float(2 * z * L)
    # print("F1: %.4f" % (F1))

    G1 = float(
        math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2) + math.pow(a, 2) + math.pow(L, 2) + (2 * y * a) - (
            math.pow(l, 2)))
    # print("G1: %.4f" % (G1))

    E2 = float(-L * (((math.sqrt(3)) * (x + b)) + y + c))
    # print("E2: %.4f" % (E2))

    F2 = float(2 * z * L)
    # print("F2: %.4f" % (F2))

    G2 = float(math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2) + math.pow(b, 2) + math.pow(c, 2) + math.pow(L, 2) + (
            2 * ((x * b) + (y * c))) - math.pow(l, 2))
    # print("G2: %.4f" % (G2))

    E3 = float(L * (((math.sqrt(3)) * (x - b)) - y - c))
    # print("E3: %.4f" % (E3))

    F3 = float(2 * z * L)
    # print("F3: %.4f" % (F3))

    G3 = float(math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2) + math.pow(b, 2) + math.pow(c, 2) + math.pow(L, 2) + (
            2 * (-(x * b) + (y * c))) - math.pow(l, 2))
    # print("G3: %.4f" % (G3))

    # print
    #####################################################################
    # EFG : Pre-Processing  the variables:
    E1Sqr = math.pow(E1, 2)
    F1Sqr = math.pow(F1, 2)
    G1Sqr = math.pow(G1, 2)
    E2Sqr = math.pow(E2, 2)
    F2Sqr = math.pow(F2, 2)
    G2Sqr = math.pow(G2, 2)
    E3Sqr = math.pow(E3, 2)
    F3Sqr = math.pow(F3, 2)
    G3Sqr = math.pow(G3, 2)
    #####################################################################
    EFGsum1 = (E1Sqr + F1Sqr - G1Sqr)
    EFGsum2 = (E2Sqr + F2Sqr - G2Sqr)
    EFGsum3 = (E3Sqr + F3Sqr - G3Sqr)
    #####################################################################
    if EFGsum1 < 0:
        EFG_1 = -(cmath.sqrt(EFGsum1)).imag
    else:
        EFG_1 = math.sqrt(EFGsum1)

    if EFGsum2 < 0:
        EFG_2 = -(cmath.sqrt(EFGsum2)).imag
    else:
        EFG_2 = math.sqrt(EFGsum2)

    if EFGsum3 < 0:
        EFG_3 = -(cmath.sqrt(EFGsum3)).imag
    else:
        EFG_3 = math.sqrt(EFGsum3)

    # Inverse Kinematics Solution 1:
    T11 = float((-(F1) + EFG_1) / (G1 - E1))
    theta11 = 2 * (math.degrees(math.atan(T11)))

    T12 = float((-(F2) + EFG_2) / (G2 - E2))
    theta12 = 2 * (math.degrees(math.atan(T12)))

    T13 = float((-(F3) + EFG_3) / (G3 - E3))
    theta13 = 2 * (math.degrees(math.atan(T13)))

    print(
        "###########################################################")
    print("Validating Solution 1:")
    if (((theta11 > 0) and (theta11 < 180)) and ((theta12 > 0) and (theta12 < 180)) and (
            (theta13 > 0) and (theta13 < 180))):
        print("Solution 1:")
        ValidSol1 = True
        print("Theta 11: %d" % (theta11))
        print("Theta 12: %d" % (theta12))
        print("Theta 13: %d" % (theta13))
        print
        # Checking for current angle status:
        if (currentTheta11 != theta11):
            StepTheta11 = ((currentTheta11 - theta11) * 4.444)
            StepTheta11 = int(round(StepTheta11))
        else:
            StepTheta11 = int(0)

        if (currentTheta12 != theta12):
            StepTheta12 = ((currentTheta12 - theta12) * 4.444)
            StepTheta12 = int(round(StepTheta12))
        else:
            StepTheta12 = int(0)

        if (currentTheta13 != theta13):
            StepTheta13 = ((currentTheta13 - theta13) * 4.444)
            StepTheta13 = int(round(StepTheta13))
        else:
            StepTheta13 = int(0)

        # Reset current theta
        currentTheta11 = theta11
        currentTheta12 = theta12
        currentTheta13 = theta13

        StepTheta11 = StepTheta11 * -1
        StepTheta12 = StepTheta12 * -1
        StepTheta13 = StepTheta13 * -1

        print("StepTheta 11: %d" % (StepTheta11))
        print("StepTheta 12: %d" % (StepTheta12))
        print("StepTheta 13: %d" % (StepTheta13))

    else:
        print("Solution 1 is invalid")
        ValidSol1 = False
    print
    #####################################################################
    # Inverse Kinematics Solution 2:
    T21 = float((-(F1) - EFG_1) / (G1 - E1))
    theta21 = 2 * (math.degrees(math.atan(T21)))

    T22 = float((-(F2) - EFG_2) / (G2 - E2))
    theta22 = 2 * (math.degrees(math.atan(T22)))

    T23 = float((-(F3) - EFG_3) / (G3 - E3))
    theta23 = 2 * (math.degrees(math.atan(T23)))
    print(
        "###########################################################")
    print("Validating Solution 2:")
    if (((theta21 > 0) and (theta21 < 180)) and ((theta22 > 0) and (theta22 < 180)) and (
            (theta23 > 0) and (theta23 < 180))):
        print("Solution 2:")
        ValidSol2 = True
        print("Theta 21: %d" % (theta21))
        print("Theta 22: %d" % (theta22))
        print("Theta 23: %d" % (theta23))
        print
        # Checking for current angle status
        if (currentTheta21 != theta21):
            StepTheta21 = ((currentTheta21 - theta21) * 4.444)
            StepTheta21 = int(round(StepTheta21))
        else:
            StepTheta21 = int(0)

        if (currentTheta22 != theta22):
            StepTheta22 = ((currentTheta22 - theta22) * 4.444)
            StepTheta22 = int(round(StepTheta22))
        else:
            StepTheta22 = int(0)

        if (currentTheta23 != theta23):
            StepTheta23 = ((currentTheta23 - theta23) * 4.444)
            StepTheta23 = int(round(StepTheta23))
        else:
            StepTheta23 = int(0)

        # Reset current theta
        currentTheta21 = theta21
        currentTheta22 = theta22
        currentTheta23 = theta23

        StepTheta21 = StepTheta21 * -1
        StepTheta22 = StepTheta22 * -1
        StepTheta23 = StepTheta23 * -1

        # Solution 2 - Steptheta:
        print("StepTheta 21: %d" % (StepTheta21))
        print("StepTheta 22: %d" % (StepTheta22))
        print("StepTheta 23: %d" % (StepTheta23))
    else:
        ValidSol2 = False
        print("Solution 2 is invalid")
        print(
            "###########################################################")
    print
    #####################################################################

    solnor2 = 0

    #If both the solutions are correct. We have to figure out which is the best solution. we will check for the variance for the current position and previous position. Based the least possible vairance we will choose the solution. To do that, each theta is compared with the previous theta's and some score is alloted for the theta variance

    solScorePoint1 = 0
    solScorePoint2 = 0
    if ((ValidSol1 is True) and (ValidSol2 is True)):
        #solnor2 = int(input("Choose solution 1 or 2 <Enter '1' or '2'>: "))
        if ((abs(currentTheta11 - theta11)) < (abs(currentTheta21 - theta21))):
            solScorePoint1 += 1
        else:
            solScorePoint2 += 1

        if ((abs(currentTheta12 - theta12)) < (abs(currentTheta22 - theta22))):
            solScorePoint1 += 1
        else:
            solScorePoint2 += 1

        if ((abs(currentTheta13 - theta13)) < (abs(currentTheta23 - theta23))):
            solScorePoint1 += 1
        else:
            solScorePoint2 += 1

        if (solScorePoint1 >= solScorePoint2):
            solnor2 = 1
        else:
            solnor2 = 2

    if solnor2 == 1:
        ValidSol2 = False
    if solnor2 == 2:
        ValidSol1 = False
    print("Sending data to arduino...")

    if ValidSol1 is True:
        TSol1 = "a" + str(StepTheta11) + "b" + str(StepTheta12) + "c" + str(StepTheta13) + "?"
        print("TSol1: %s" % (TSol1))
        for i in TSol1:
            ArduinoSerial.write(bytes(i.encode('ascii')))
            time.sleep(0.05)

    if ValidSol2 is True:
        TSol2 = "a" + str(StepTheta21) + "b" + str(StepTheta22) + "c" + str(StepTheta23) + "?"
        print("TSol2: %s" % (TSol2))
        for i in TSol2:
            ArduinoSerial.write(bytes(i.encode('ascii')))
            time.sleep(0.05)

    if ((ValidSol1 is False) and (ValidSol2 is False)):
        TSol3 = "a" + str(0) + "b" + str(0) + "c" + str(0) + "?"
        print("TSol3: %s" % (TSol3))
        for i in TSol3:
            ArduinoSerial.write(bytes(i.encode('ascii')))
            time.sleep(0.05)
        SampleListener()
		
		
		
    #####################################################################
    while (ArduinoSerial.inWaiting() > 0):
        ArduinoData = ArduinoSerial.readline()
        print("ArduinoData: %s" % (ArduinoData))
        time.sleep(0.05)

    print(
        "#####################################################################")
    print(
        "#####################################################################")
    time.sleep(1)
if __name__ == "__main__":
    main()
	
	#####################################################################
	#####################################################################
	#####################################################################