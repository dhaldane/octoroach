from lib import command
import time,sys
import serial
import shared

from or_helpers import *
from hall_helpers import *


###### Operation Flags ####
SAVE_DATA = True
RESET_ROBOT = True   

def main():    
    setupSerial()

    if SAVE_DATA:
        shared.dataFileName = findFileName();
        print "Data file:  ", shared.dataFileName

    #sendEcho("echo test")
    #time.sleep(0.2)

    if RESET_ROBOT:
        print "Resetting robot..."
        resetRobot()
        time.sleep(0.5)
        
    # Send robot a WHO_AM_I command, verify communications
    queryRobot()
    

    #Hall gains format:
    #  [ Kp , Ki , Kd , Kaw , ff         Kp , Ki , Kd , Kaw , ff ]
    #    ----------LEFT---------         ----------RIGHT----------
    
    motorgains = [300,0,300,0,600, 300,0,300,0,600]
    throttle = [0,0]
    duration = 3000
    delta = [8,13,8,13]  # adds up to 42 counts- should be 42.6
    intervals = [200, 75, 200, 75] 
    
    hparams = hallParams(motorgains, throttle, duration, delta, intervals)
    
    setHallGains(motorgains)
    setVelProfile(hparams)
    hallZeroPos()
    
    
    #Timing settings
    shared.leadinTime = 500;
    shared.leadoutTime = 500;
    shared.runtime = hparams.duration
    
    numSamples = int(ceil(300 * (shared.runtime + shared.leadinTime + shared.leadoutTime) / 1000.0))
    shared.imudata = [ [] ] * numSamples
    
    #Flash must be erased to save new data
    if SAVE_DATA:
        eraseFlashMem(numSamples)

    # Pause and wait to start run, including leadin time
    raw_input("Press enter to start run ...")
    
    
    # Trigger telemetry save, which starts as soon as it is received
    if SAVE_DATA:
        startTelemetrySave(numSamples)

    time.sleep(shared.leadinTime / 1000.0)
    
    ### START MOVEMENT HERE
    hallProceed(hparams)
    
    #time.sleep for length of experiment is done inside downloadTelemetry()

    if SAVE_DATA:
        downloadTelemetry(numSamples)

    #Wait for Ctrl+C to exit; this is done in case other messages come in
    #from the robot, which are handled by callbackfunc
    #print "Ctrl + C to exit"
    
    #while True:
    #    try:
    #        time.sleep(1)
            #print ".",
    #    except KeyboardInterrupt:
    #        break

    shared.xb.halt()
    shared.ser.close()


    print "Done"

#Provide a try-except over the whole main function
# for clean exit. The Xbee module should have better
# provisions for handling a clean exit, but it doesn't.
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print "\nRecieved Ctrl+C, exiting."
        shared.xb.halt()
        shared.ser.close()
    except Exception as args:
        print "\nGeneral exception:",args
        print "Attemping to exit cleanly..."
        shared.xb.halt()
        shared.ser.close()
        sys.exit()
    except serial.serialutil.SerialException:
        shared.xb.halt()
        shared.ser.close()
