# -*- coding: utf-8 -*-
"""
Created on Fri Oct 27 16:09:01 2017
@author: Jun Hao
"""

import sys
import time
import pandas as pd
import numpy as np
import serial
import array

# Implement simple timer
current_milli_time = lambda: int(round(time.time() * 1000))
timetotalsegment = 0

# Print current time on terminal
from datetime import datetime
print (str(datetime.now()))

# Config.ini
reshapeBy = 50 # Set number of inputs per sample for Machine Learning
arduinoPort = "/dev/ttyACM0"
debugLoops = 3
mainLoops = 40 # 1 minute = 60s = 120 sets of data
skipCalibration = True
#sys.stdout = open("OutputComb.txt", "w") # Set print command to print to file

# Variable Declarations
flag = 1
isHandshakeDone = False
calibrated = False
debugFailCount = 0
ignoreLoopCount = 0
loopCount = 0
successCount = 0
checkSumFailCount = 0
IDFailCount = 0
newAccID = 0
oldAccID = 0
oldTime = newTime = current_milli_time()
hashcount = 0
msgCheckSum = 0
checkSum = 0
errorFlag = 0
calibrated = False
x0cal = 0
y0cal = 0
z0cal = 0
x1cal = 0
y1cal = 0
z1cal = 0
x2cal = 0
y2cal = 0
z2cal = 0

# Static Declarations
handshake = ("\r\nH").encode()
acknoledged = ("\r\nA").encode()
clear = ("\r\nAAAAAAAAAA").encode()
resend = ("\r\nR").encode()
reshapedBy = int(reshapeBy*9)
cols = [list(range(1, (9*reshapeBy)+1))] # Declare column headers 1-450

# Declare empty DataFrame
fullDF = pd.DataFrame(columns=cols)

# Initialize Arduino connection and perform handshake
print("Connecting to Raspberry Pi")
ser = serial.Serial(arduinoPort, baudrate=115200, timeout=3.0)
sys.stdout.write("\033[F") # Cursor up one line
sys.stdout.write("\033[K") # Clear line
print ("Raspberry Pi Connected")
while (isHandshakeDone == False):
        ser.write(handshake) # Initiate handshake on Arduino
        print("H sent, awaiting response")
        response = ser.read().decode()
        if response == ('A'):
            print("Response verified, handshake complete")
            isHandshakeDone = True
            ser.write(acknoledged)
            ser.readline() # Clear the screaming "AAAAAAAAAAAAAAAAA"
            time.sleep(0.5)
            print("Starting in 3 seconds")
            time.sleep(0.5)
            sys.stdout.write("\033[F") # Cursor up one line
            sys.stdout.write("\033[K") # Clear line
            print ("Starting in 2 seconds")
            time.sleep(0.5)
            ser.write(acknoledged)
            ser.readline()
            sys.stdout.write("\033[F") # Cursor up one line
            sys.stdout.write("\033[K") # Clear line
            print ("Starting in 1 seconds")
            time.sleep(0.5)
            sys.stdout.write("\033[F") # Cursor up one line
            sys.stdout.write("\033[K") # Clear line
            print ("Begin")
            ser.write(acknoledged) # Intentionally increase message ID to #2 to test ID error checking in system test section
        else:
            ser.write(handshake)
            print(response)
            ser.write(handshake)

# Calibration (NOT IMPLEMENTED)
if (skipCalibration == False):
    startTime = current_milli_time()
    while (calibrated == False):
        # 1. read data
        # 2. check if any value exceed limits -> WARN WORN INCORRECT -> reset to step 1
        # 3. check if large fluctuation from any previous datas -> WARN LARGE FLUCTUATION -> reset to step 1
        # 3b. else save data and loop until count = 200 -> take average of 200 sets of data to be calibrateCandidate1
        # 4. ask user to move about and reset position to neutral -> give 5 seconds before starting
        # 5. restart from 1 for calibrateCandidate2
        # 6. check if calibrateCandidate1 is close to calibrateCandidate2, if yes, take the average and save calibration data
        print("Calibration took (ms): ", (current_milli_time()-startTime))

# Ignore early readings (System test)
print("Begin System Test")
startTime = loopTime = current_milli_time()
while (ignoreLoopCount < debugLoops):
    loopTime = readTime = current_milli_time()
    message = ser.readline() # Read message from Arduino
    readEndTime = current_milli_time()

    ser.write(acknoledged) # Instruct Arduino to prepare next set of data

    message = message.decode() # Convert to string to manipulate data
    print("Message:", message)
    newAccID = int(message.split(',', 1)[0]) # Extract message ID
    msgCheckSum = int((message.rsplit(',', 1)[1])[:-2]) # Extract message checksum
    message = message.rsplit(',', 1)[0] # Remove checksum from message
    byteMessage = array.array('b', message.encode()) # Convert back to byteMessage to generate hash

    if (newAccID == (oldAccID + 1)): # Check if ID Incremented
        oldAccID = newAccID
        while (hashcount < len(byteMessage)): # Produce checksum from received data
            checkSum ^= int(byteMessage[hashcount])
            hashcount += 1

        if (checkSum == msgCheckSum): #Check if checksums matches
            print('Checksum matches')
        else: # Checksums do not match
            debugFailCount += 1
            print('Checksums error!', "Message Checksum:", msgCheckSum, "Generated Checksum:", checkSum)
            print(' ')

    elif (newAccID == oldAccID):
        debugFailCount += 1
        print('Same message ID received!')
        print(' ')
    else:
        debugFailCount += 1
        print('ID error!', 'oldAccID:', oldAccID, 'newAccID:', newAccID)
        print(' ')

    ignoreLoopCount += 1
    checkSum = 0
    hashcount = 0
    oldAccID = newAccID
    print("In debug loop:", ignoreLoopCount, "Reading took:", readEndTime-readTime, "ms", "Others took:", current_milli_time()-readEndTime)

print("Average debug loop duration (ms): ", ((current_milli_time()-startTime)/debugLoops), "with", debugFailCount, "errors")

print("Starting in 3 seconds")
time.sleep(0.5)
sys.stdout.write("\033[F") # Cursor up one line
sys.stdout.write("\033[K") # Clear line
print ("Starting in 2 seconds")
time.sleep(0.5)
sys.stdout.write("\033[F") # Cursor up one line
sys.stdout.write("\033[K") # Clear line
print ("Starting in 1 seconds")
time.sleep(0.5)
sys.stdout.write("\033[F") # Cursor up one line
sys.stdout.write("\033[K") # Clear line
print ("Begin")

# Read (Main Loop)
print(' ')
print('SYSTEM LIVE')
print(' ')
startTime = current_milli_time()

while (loopCount < mainLoops):
    startTime = current_milli_time()
    loopCount += 1
    
    message = ser.readline() # Read message from Arduino
    ser.write(acknoledged) # Instruct Arduino to prepare next set of data
    
    message = message.decode() # Convert to string to manipulate data
    newAccID = int(message.split(',', 1)[0]) # Extract message ID
    volt = int(message.rsplit(',', 3)[1])
    amp = int(message.rsplit(',', 2)[1])
    msgCheckSum = int((message.rsplit(',', 1)[1])[:-2]) # Extract message checksum
    message = message.rsplit(',', 1)[0] # Remove checksum from message
    byteMessage = array.array('b', message.encode()) # Convert back to byteMessage to generate hash

    if (newAccID == (oldAccID + 1)): # Check if ID Incremented
        while (hashcount < len(byteMessage)): # Produce checksum from received data
            checkSum ^= int(byteMessage[hashcount])
            hashcount += 1

        if (checkSum == msgCheckSum): #Check if checksums matches
            message = message.rsplit(',', 2)[0] # Remove volt and amp from message
            message = message.split(',', 1)[1] # Remove ID from message
            #message += ',10' # 'standing', 'wavehands', 'busdriver', 'frontback', 'sidestep', 'jumping', 'jumpingjack', 'turnclap', 'squatturnclap', 'windowcleaning', 'windowcleaner360', 'logout'
            messagenp = np.fromstring(message[0:(len(message))], dtype=int, sep=",")
            #print(messagenp)
            #messagenp = messagenp.reshape(1,-1)

            messagepd = pd.DataFrame(data=messagenp.reshape(-1, (len(messagenp))), index=['1'], columns=cols)
            messagepd['451']='standing'
            #print(messagepd)
            fullDF = fullDF.append(messagepd, ignore_index = True)

            successCount += 1

        else: # Checksums do not match
            #ser.write(acknoledged) # Send request for resend of data from Arduino
            checkSumFailCount += 1
            errorFlag = True
            print('Checksums error!', "Message Checksum:", msgCheckSum, "Generated Checksum:", checkSum)
            print("Message:", message)
            print(' ')

    elif (newAccID == oldAccID): # Repeated message recieved
        IDFailCount += 1
        errorFlag = True
        print('ID error!', 'Same message received!')
        print(' ')
    else: # Unexpected/corrupt ID recieved
        IDFailCount += 1
        errorFlag = True
        print('ID error!', 'oldAccID:', oldAccID, 'newAccID:', newAccID)
        print("Message:", message)
        print(' ')

    # Reset values
    oldAccID = newAccID
    checkSum = 0
    hashcount = 0

    # Show user number of loops
    if (loopCount%5== 0):
        print('Successes:', successCount, '| ID errors:', IDFailCount,'| Checksum errors:', checkSumFailCount)


print('Average main loop duration (ms):', ((current_milli_time()-startTime)/mainLoops))
print(fullDF)

# Save raw data to csv file
fullDF.to_csv('Raw_Data_0.csv', sep=',')
