from TRSensor import TRSensor
from Motor import PicoGo
import time


print("\nTRSensor Test Program ...\r\n")
time.sleep(3)
M = PicoGo()
TRS=TRSensor()
for i in range(100):
    if(i<25 or i>= 75):
        M.setMotor(30,-30)
    else:
        M.setMotor(-30,30)
    TRS.calibrate()
print("\ncalibrate done\r\n")
print(TRS.calibratedMin)
print(TRS.calibratedMax)
print("\ncalibrate done\r\n")
maximum = 100
integral = 0
last_proportional = 0

while True:
    #print(TRS.readCalibrated())
    #print(TRS.readLine())
    position,Sensors = TRS.readLine()
    #time.sleep(0.1)
    if((Sensors[0] + Sensors[1] + Sensors[2]+ Sensors[3]+ Sensors[4]) > 4000):
        M.setMotor(0,0)
    else:
        # The "proportional" term should be 0 when we are on the line.
        proportional = position - 2000

        # Compute the derivative (change) and integral (sum) of the position.
        derivative = proportional - last_proportional
        integral += proportional

        # Remember the last position.
        last_proportional = proportional
        
        '''
        // Compute the difference between the two motor power settings,
        // m1 - m2.  If this is a positive number the robot will turn
        // to the right.  If it is a negative number, the robot will
        // turn to the left, and the magnitude of the number determines
        // the sharpness of the turn.  You can adjust the constants by which
        // the proportional, integral, and derivative terms are multiplied to
        // improve performance.
        '''
        power_difference = proportional/30  + derivative*2;  

        if (power_difference > maximum):
            power_difference = maximum
        if (power_difference < - maximum):
            power_difference = - maximum
        
        if (power_difference < 0):
            M.setMotor(maximum + power_difference, maximum)
        else:
            M.setMotor(maximum, maximum - power_difference)

