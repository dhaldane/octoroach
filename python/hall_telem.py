from lib import command
import time,sys
import serial
import shared
import msvcrt, sys

from or_helpers import *
from hall_helpers import *

def main():
##  Setup and Initalization

    ###### Operation Flags ####
    SAVE_DATA = False
    RESET_ROBOT = True   

    motorgains = [300,0,300,0,600, 300,0,300,0,600]
    shared.motorGains = motorgains
    throttle = [0,0]
    duration = 1700
    delta = [8,8,8,8]  # adds up to 42 counts- should be 42.6
    intervals = [30, 30, 30, 30]
    cycle = 125
    setupSerial()

    ##Ramp parameters
    rDelta =     [10,10,20,20,20,20,20,25,25,25,25,25,30,30,25,25]
    rIntervals = [50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50]
    

    
    if RESET_ROBOT:
        print "Resetting robot..."
        resetRobot()
        time.sleep(0.5)
    queryRobot()

    hparams = hallParams(motorgains, throttle, duration, delta, intervals)
    setHallGains(motorgains)
    setVelProfile(hparams)
    hallZeroPos()

    rparams = rampParams(motorgains, throttle, duration, rDelta, rIntervals)
    setRampProfile(rparams)

    shared.leadinTime = 500;
    shared.leadoutTime = 500;
    shared.runtime = hparams.duration

    numSamples = int(ceil(300 * (shared.runtime + shared.leadinTime + shared.leadoutTime) / 1000.0))
    shared.imudata = [ [] ] * numSamples

    menu()

    while True:
        print '>',
        keypress = msvcrt.getch()
        if keypress == 'h':
            menu() 
        elif keypress =='m':
            SAVE_DATA = not(SAVE_DATA)
            print 'Telemetry recording', SAVE_DATA
        elif keypress =='n':
            sendWhoAmI()      
        elif (keypress == 'p'):
            time.sleep(0.5)
            numSamples = int(ceil(300 * (shared.runtime + shared.leadinTime + shared.leadoutTime) / 1000.0))
            shared.imudata = [ [] ] * numSamples
            if SAVE_DATA:
                eraseFlashMem(numSamples)
                startTelemetrySave(numSamples)
            time.sleep(shared.leadinTime / 1000.0)
            hallProceed(hparams)

            if SAVE_DATA:
                path     = 'Data/'
                name     = 'trial'
                datetime = time.localtime()
                dt_str   = time.strftime('%Y.%m.%d_%H.%M.%S', datetime)
                root     = path + dt_str + '_' + name
                shared.dataFileName = root + '_imudata.txt'
                print "Data file:  ", shared.dataFileName
                downloadTelemetry(numSamples)

        elif keypress == 'r':
            
            resetRobot()
            time.sleep(0.5)
            print 'Resetting robot'
            setHallGains(motorgains)
            
        elif keypress == 't':
            
            print 'cycle='+str(cycle)+' duration='+str(duration)+'. New duration:',
            duration = int(raw_input())
            hparams = hallParams(motorgains, throttle, duration, delta, intervals)
            setVelProfile(hparams)
            shared.runtime = hparams.duration
            
        elif keypress == 'a':
            
            print 'cycle='+str(cycle)+' duration='+str(duration)+'. New Frequency (Hz):',
            cycle = 1000/int(raw_input())
            intervals = [cycle/4, cycle/4, cycle/4, cycle/4]
            tempsum = intervals[0]+intervals[1]+intervals[2]+intervals[3]
            intervals[3]=intervals[3]+cycle - tempsum
            hparams = hallParams(motorgains, throttle, duration, delta, intervals)
            setVelProfile(hparams)
            
        elif keypress == 's':
            
            print 'Enter Delta values as CSV',
            x = raw_input()
            if len(x):
                temp = map(int,x.split(','))
                for i in range(0,16):
                    rDelta[i] = temp[i]
            else:
                print 'Not enough values'
            rparams = rampParams(motorgains, throttle, duration, rDelta, rIntervals)
            setRampProfile(rparams)
            shared.rampDelta = rDelta
            
        elif keypress == 'z':
            
            hallZeroPos()
            print 'read motorpos and zero'
            
        elif (keypress == 'q') or (ord(keypress) == 26):
            
            print "Exit."
            shared.xb.halt()
            shared.ser.close()
            sys.exit(0)
            
        else:
            print "** unknown keyboard command** \n"
            menu()
    


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
