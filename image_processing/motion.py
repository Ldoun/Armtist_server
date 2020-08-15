from armangle import armPositions
from datetime import datetime
#from edge_detection import zero_one
import time

def loadMotionQueue():

    SERVO_BASE_FORWARD_DEGREES = 0.0
    SERVO_A_UP_DEGREES = 90.0
    SERVO_B_UP_DEGREES = 0.0
    SERVO_C_UP_DEGREES = 0.0

    queueCount = 0
    lastQueueCount = 0
    tempPosAng = 0.0

    '''motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES
    motionQueue[queueCount][1] = SERVO_A_UP_DEGREES 
    motionQueue[queueCount][2] = SERVO_B_UP_DEGREES
    motionQueue[queueCount][3] = SERVO_C_UP_DEGREES
    queueCount+=1'''

    #마지막 위치
    motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 90.0
    motionQueue[queueCount][1] = SERVO_A_UP_DEGREES + 5.0
    motionQueue[queueCount][2] = SERVO_B_UP_DEGREES  +65.0
    motionQueue[queueCount][3] = SERVO_C_UP_DEGREES + 90.0
    queueCount+=1

    #다음 위치 위에꺼랑 비교해서 다른게 움직임 아래꺼 같은경우 1번모터만 움직임 그래서 초반에 1번이  많이 나온거고
    motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 90.0 # cup midpoint
    motionQueue[queueCount][1] = SERVO_A_UP_DEGREES - 35.0
    motionQueue[queueCount][2] = SERVO_B_UP_DEGREES + 65.0
    motionQueue[queueCount][3] = SERVO_C_UP_DEGREES + 90.0
    queueCount+=1

    #
    motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 90.0 #rotated
    motionQueue[queueCount][1] = SERVO_A_UP_DEGREES
    motionQueue[queueCount][2] = SERVO_B_UP_DEGREES
    motionQueue[queueCount][3] = SERVO_C_UP_DEGREES
    queueCount+=1

    # get some paint 붓 물감 찍기
    motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 52.8 # paint midpoint
    motionQueue[queueCount][1] = SERVO_A_UP_DEGREES
    print(motionQueue[queueCount][1])
    motionQueue[queueCount][2] = SERVO_B_UP_DEGREES
    motionQueue[queueCount][3] = SERVO_C_UP_DEGREES
    queueCount+=1
    motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 52.8 # endpoint in paint
    motionQueue[queueCount][1] = SERVO_A_UP_DEGREES + 0.8
    motionQueue[queueCount][2] = SERVO_B_UP_DEGREES + 62.0
    motionQueue[queueCount][3] = SERVO_C_UP_DEGREES + 90.4
    queueCount+=1
    motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 52.8 # paint midpoint
    motionQueue[queueCount][1] = SERVO_A_UP_DEGREES
    motionQueue[queueCount][2] = SERVO_B_UP_DEGREES
    motionQueue[queueCount][3] = SERVO_C_UP_DEGREES
    queueCount+=1
    motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES # zero
    motionQueue[queueCount][1] = SERVO_A_UP_DEGREES
    motionQueue[queueCount][2] = SERVO_B_UP_DEGREES
    motionQueue[queueCount][3] = SERVO_C_UP_DEGREES
    queueCount+=1
    motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES # zero
    motionQueue[queueCount][1] = SERVO_A_UP_DEGREES - 10
    motionQueue[queueCount][2] = SERVO_B_UP_DEGREES
    motionQueue[queueCount][3] = SERVO_C_UP_DEGREES
    queueCount+=1

    # see what, if anything, is in the image array     
    for qq in range(576):# 근데 이러면 이미지 해상도 올려도 되겠다
        if (imagePacketFinal[qq]): # if the pixel is "on" then load that position from the .h file into the queue  픽셀이 1이면 이미지 프로세싱 함수 참고
            #print('here')
            motionQueue[queueCount][0] = armPositions[qq][0]
            motionQueue[queueCount][1] = armPositions[qq][1]
           # print(motionQueue[queueCount][1],'|',queueCount)
            motionQueue[queueCount][2] = armPositions[qq][2]
            motionQueue[queueCount][3] = armPositions[qq][3]
            queueCount+=1
            tempPosAng = armPositions[qq][2] # get the value for thetaB  뭔지 모르겠다
            tempPosAng += 20.0 # move back 20 degrees
            motionQueue[queueCount][0] = armPositions[qq][0]
            motionQueue[queueCount][1] = armPositions[qq][1]
            motionQueue[queueCount][2] = tempPosAng
            motionQueue[queueCount][3] = armPositions[qq][3]
            queueCount+=1
            '''if (abs(queueCount - lastQueueCount) >= 30): # every 30 queue settings (which means 15 pixels have been painted) load a reach paint and return to zero sequence  30개의 픽셀을 칠한후 페인트 부족할까봐 가지러감
                
                lastQueueCount = queueCount # adding paint!
                
                motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES # load painting sequence (zero, midpoint, endpoint, midpoint, zero)
                motionQueue[queueCount][1] = SERVO_A_UP_DEGREES
                motionQueue[queueCount][2] = SERVO_B_UP_DEGREES
                motionQueue[queueCount][3] = SERVO_C_UP_DEGREES
                queueCount+=1
                motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 52.8 # midpoint
                motionQueue[queueCount][1] = SERVO_A_UP_DEGREES
                motionQueue[queueCount][2] = SERVO_B_UP_DEGREES
                motionQueue[queueCount][3] = SERVO_C_UP_DEGREES
                queueCount+=1
                motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 52.8  # endpoint in paint
                motionQueue[queueCount][1] = SERVO_A_UP_DEGREES - 0.8
                motionQueue[queueCount][2] = SERVO_B_UP_DEGREES - 62.0
                motionQueue[queueCount][3] = SERVO_C_UP_DEGREES - 90.4
                queueCount+=1
                motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 52.8 # midpoint
                motionQueue[queueCount][1] = SERVO_A_UP_DEGREES
                motionQueue[queueCount][2] = SERVO_B_UP_DEGREES
                motionQueue[queueCount][3] = SERVO_C_UP_DEGREES
                queueCount+=1
                motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES # zero
                motionQueue[queueCount][1] = SERVO_A_UP_DEGREES
                motionQueue[queueCount][2] = SERVO_B_UP_DEGREES
                motionQueue[queueCount][3] = SERVO_C_UP_DEGREES
                queueCount+=1
                motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES # tip back
                motionQueue[queueCount][1] = SERVO_A_UP_DEGREES + 35
                motionQueue[queueCount][2] = SERVO_B_UP_DEGREES
                motionQueue[queueCount][3] = SERVO_C_UP_DEGREES
                queueCount+=1
                            #  END IF queueCount'''
        
    # once pixel count is done, add go to sleep sequence  종료 시퀸스
    motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES # start with zero postion
    motionQueue[queueCount][1] = SERVO_A_UP_DEGREES
    motionQueue[queueCount][2] = SERVO_B_UP_DEGREES
    motionQueue[queueCount][3] = SERVO_C_UP_DEGREES
    queueCount+=1
    motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 90.0 #rotated
    motionQueue[queueCount][1] = SERVO_A_UP_DEGREES
    motionQueue[queueCount][2] = SERVO_B_UP_DEGREES
    motionQueue[queueCount][3] = SERVO_C_UP_DEGREES
    queueCount+=1
    motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 90.0 #midpoint
    motionQueue[queueCount][1] = SERVO_A_UP_DEGREES + 35.0
    motionQueue[queueCount][2] = SERVO_B_UP_DEGREES - 65.0
    motionQueue[queueCount][3] = SERVO_C_UP_DEGREES - 90.0
    queueCount+=1
    motionQueue[queueCount][0] = SERVO_BASE_FORWARD_DEGREES - 90.0 #endpoint in water cup
    motionQueue[queueCount][1] = SERVO_A_UP_DEGREES - 5.0
    motionQueue[queueCount][2] = SERVO_B_UP_DEGREES - 65.0
    motionQueue[queueCount][3] = SERVO_C_UP_DEGREES - 90.0
    queueCount+=1
    #print(motionQueue)

def run_motionque():
    queueHasData = True
    curRow = 0

    while (queueHasData):
        #print(curRow)
       # print(motionQueue[curRow+1][1])
        if (motionQueue[curRow+1][1] >= 40.0): # check if angle A is ever too low or zero, which means the next value is an empty one
            moveBetweenShapes(motionQueue[curRow][0], motionQueue[curRow][1], motionQueue[curRow][2], motionQueue[curRow][3],motionQueue[curRow+1][0], motionQueue[curRow+1][1], motionQueue[curRow+1][2], motionQueue[curRow+1][3])
            curRow+=1 #move to the next shape in the queue      
        else :# we've finished the run
            #print('here')
            queueHasData = False                   

def moveBetweenShapes(thetaBaseStart,thetaAStart, thetaBStart, thetaCStart, thetaBaseStop, thetaAStop, thetaBStop, thetaCStop):
    # smoothly move from one arm shape to the next, in the quickest time possible, and with all motors arriving at the desired shape at the same time  모양 그리는 함수 인듯 수학 공식 포함
    #changed
    global SERVO_A
    SERVO_A=1
    global SERVO_B 
    SERVO_B=2
    global SERVO_C 
    SERVO_C=3
    global SERVO_BASE 
    SERVO_BASE=0
    
    doneMoving = False
    baseAtTarget = False # need to begin movement at theta start
    motAAtTarget = False
    motBAtTarget = False
    motCAtTarget = False
    
    tempBaseAngle = thetaBaseStart
    tempMotAAngle = thetaAStart
    tempMotBAngle = thetaBStart
    tempMotCAngle = thetaCStart

    deltaThetaBaseDir = 0 # the sign of a given deltaTheta value
    deltaThetaADir = 0
    deltaThetaBDir = 0
    deltaThetaCDir = 0

    deltaBaseInterval = 0
    deltaMotAInterval = 0
    deltaMotBInterval = 0
    deltaMotCInterval = 0
    totalTime = 0
    
    deltaThetaBase = abs(thetaBaseStart - thetaBaseStop)
    deltaThetaA = abs(thetaAStart - thetaAStop)
    deltaThetaB = abs(thetaBStart - thetaBStop)
    deltaThetaC = abs(thetaCStart - thetaCStop)
    maxAngularResoltion = 1.0#  move no greater than one degree increments (this value cannot be changed without modifying other math
    maxDelay = 20 # move no faster that one degree per every 20 milliseconds in either direction

    if (thetaBaseStart - thetaBaseStop < 0):
        deltaThetaBaseDir = 1
    if (thetaBaseStart - thetaBaseStop > 0):
        deltaThetaBaseDir = -1
    if (thetaAStart - thetaAStop < 0):
            deltaThetaADir = 1
    if (thetaAStart - thetaAStop > 0):
            deltaThetaADir = -1
    if (thetaBStart - thetaBStop < 0): 
        deltaThetaBDir = 1
    if (thetaBStart - thetaBStop > 0): 
        deltaThetaBDir = -1
    if (thetaCStart - thetaCStop < 0): 
        deltaThetaCDir = 1
    if (thetaCStart - thetaCStop > 0): 
        deltaThetaCDir = -1

    greaterResult = 0.0

    if (deltaThetaBase > deltaThetaA):
        greaterResult = deltaThetaBase# // lets find the maximum angular change 
    else:
        greaterResult = deltaThetaA#; } // we don't need to worry if they're equal, since we ultimately just want the largest deltaTheta value
    if (greaterResult > deltaThetaB):
        # greaterResult = greaterResult;
        pass
    else:
        greaterResult = deltaThetaB
    if greaterResult > deltaThetaC:
        #{} // greaterResult = greaterResult;
        pass
    else :
        greaterResult = deltaThetaC  

    totalTime = int(greaterResult) * maxDelay#; // now set our total time based upon the largest change in motor postion, this way the arm moves as fast as possible
    if (deltaThetaBase > 0.0):
        deltaBaseInterval = int(float(totalTime) / deltaThetaBase)  #deltaThetaBase = abs(thetaBaseStart - thetaBaseStop) greaterResult = deltaThetaB
        
    else:
        deltaBaseInterval = 0#// the inteval between updating motor positions, don't devide by zero
    #NOTE: the galileo decides int(1.0) is equal to 0...
    if (deltaThetaA > 0.0):
        deltaMotAInterval = int(float(totalTime) / deltaThetaA)
    else:
        deltaMotAInterval = 0#;} // The Galileo doesn't seem to reliably convert between types... 
    if (deltaThetaB > 0.0):
        deltaMotBInterval = int(float(totalTime) / deltaThetaB)
    else:
        deltaMotBInterval = 0

    if (deltaThetaC > 0.0):
        deltaMotCInterval = int(float(totalTime) / deltaThetaC)
    else :
        deltaMotCInterval = 0

    print(deltaBaseInterval)
    print(deltaMotAInterval)
    print(deltaMotBInterval)
    print(deltaMotCInterval)
    #changed
    deltaBaseInterval/=200
    deltaMotAInterval/=200
    deltaMotBInterval/=200
    deltaMotCInterval/=200

    motorTime = 0 # a marker to keep track of when we need to increment the motor position
    lastBaseTime = 0
    lastMotATime = 0
    lastMotBTime = 0
    lastMotCTime = 0
    start = time.time()
    while (doneMoving == False): #update the motor angles until we've reached the desired position for all motors
        
        motorTime=round(round(time.time() - start,4),4)
       # print(motorTime)

        if  (deltaThetaBase == 0.0):# don't send any position data to the motor if it is already at the desired angle
            baseAtTarget = True
        else :#increase or decrease the motor angle until is is greater than or equal to the target position, then limit it to the thetaStop value
            if (motorTime - lastBaseTime > deltaBaseInterval):# has it been long enough since we last updated this motor postion?
                lastBaseTime = motorTime # reset our current motor timer
                tempBaseAngle += float(deltaThetaBaseDir) * maxAngularResoltion # degree should be positive or negative depending on deltaThetaDir
                if (tempBaseAngle > thetaBaseStop and deltaThetaBaseDir == 1): # don't set the motor beyond the desired angles
                    tempBaseAngle = thetaBaseStop
                    baseAtTarget = True
                
                if (tempBaseAngle < thetaBaseStop and deltaThetaBaseDir == -1):
                    tempBaseAngle = thetaBaseStop
                    baseAtTarget = True
                
                setMotor(SERVO_BASE, tempBaseAngle) # convert the angle to a pulse length for the PWM driver board
            
            # END motor BASE update

        if  (deltaThetaA == 0):# // update motor A
            motAAtTarget = True
        else :  
            #print(motorTime,lastMotATime,deltaMotAInterval)
            if (motorTime - lastMotATime > deltaMotAInterval):
                lastMotATime = motorTime
                tempMotAAngle += float(deltaThetaADir) * maxAngularResoltion
                if (tempMotAAngle > thetaAStop and deltaThetaADir == 1):
                    tempMotAAngle = thetaAStop
                    motAAtTarget = True
                    
                if (tempMotAAngle < thetaAStop and deltaThetaADir == -1):
                    tempMotAAngle = thetaAStop 
                    motAAtTarget = True
                
                setMotor(SERVO_A, tempMotAAngle)
            
            # // END motor A update

        if  (deltaThetaB == 0):# // update motor B
            motBAtTarget = True
        else :
            if (motorTime - lastMotBTime > deltaMotBInterval):
                lastMotBTime = motorTime
                tempMotBAngle += float(deltaThetaBDir) * maxAngularResoltion
                if (tempMotBAngle > thetaBStop and deltaThetaBDir == 1):
                    tempMotBAngle = thetaBStop
                    motBAtTarget = True
                
                if (tempMotBAngle < thetaBStop and deltaThetaBDir == -1):
                    tempMotBAngle = thetaBStop 
                    motBAtTarget = True
                
                setMotor(SERVO_B, tempMotBAngle)
            # // END motor B 
            
        if  (deltaThetaC == 0):# // update motor C
            motCAtTarget = True
        else :
            if (motorTime - lastMotCTime > deltaMotCInterval):
                """print('motoTime',motorTime)
                print('lastMot',lastMotCTime)
                print(deltaMotCInterval)"""
                lastMotCTime = motorTime
                tempMotCAngle += float(deltaThetaCDir) * maxAngularResoltion
                if (tempMotCAngle > thetaCStop and deltaThetaCDir == 1):
                    tempMotCAngle = thetaCStop 
                    motCAtTarget = True
                
                if (tempMotCAngle < thetaCStop and deltaThetaCDir == -1):
                    tempMotCAngle = thetaCStop
                    motCAtTarget = True
                    
                setMotor(SERVO_C, tempMotCAngle)
            # // END motor C update

        time.sleep(maxDelay/1000)# wait some time for the motors to physically catch up to the set position

        if (baseAtTarget and motAAtTarget and motBAtTarget and motCAtTarget):
            doneMoving = True


def setMotor(whichMotor, whatAngle):
  
    pulsesPerDegree = 2.5
    tempPulse = 0

    whatAngle *= 100.0#; //multiply by ten to preserve some degree of precision
    whatAngle=int(whatAngle)
    #{ // map from degrees to pulse length
    if whichMotor==SERVO_BASE:
        tempPulse = int(map2(whatAngle, 4500, -9000, 25250, 59000)) # CCW angluar max (45 degrees) to CW angular max (90 degrees) to pulses (252.5 microseconds) to (590 microseconds) 
    if whichMotor==SERVO_A:
        tempPulse = int(map2(whatAngle, 13500, 4500, 24450, 46950))
    if whichMotor==SERVO_B:
        tempPulse = int(map2(whatAngle, 2250, -11250, 41625, 7875))
    if whichMotor==SERVO_C:
        tempPulse = int(map2(whatAngle, 2250, -11250, 41925, 8175))
  #int pulseOffset = int(whatAngle * pulsePerDegree); pulseForMotorAtZero += pulseOffset; // <- This is an alternative, unused mapping function (incomplete)
  

    tempPulse /= 100#; // divide by ten to return to the proper pulse length
    #pwm.setPWM(whichMotor, 0, tempPulse) #need to change
    print('this',whichMotor,":",int(tempPulse))

    data=str(whichMotor)+str(tempPulse)
    

    return tempPulse # just to see how well the function converts the angles
    # END FUNCTION setMotor()

def map2( x,  in_min,  in_max,  out_min,  out_max) :
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def motion():   

    #작동이 이상하다 싶으면 emptyMotionQueue() 만들기
    loadMotionQueue()
    run_motionque()


imagePacketFinal = [ #02 "blocks" of 5 by 7 pixel shapes with a 0 pixel right and bottom border  24*24 배열 요따가 이미지 넣기 어쩌면 선으로 하는것 보다 형태로 안에 꽉채워서 하는게 더 좋을 수도
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,

    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,	
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0
    ]
#imagePacketFinal=zero_one()  !#

#print('image',armPositions[300])
motionQueue= [[0 for col in range(4)] for row in range(1200)]
print(motionQueue)
motion()