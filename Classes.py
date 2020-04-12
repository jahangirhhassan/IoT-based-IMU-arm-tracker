from struct import *
import numpy as np
import sys
import math
from collections import namedtuple
import time
import csv


NUMBER_OF_NODES = 2 # Number of sensor nodes in the system
pointCoor = namedtuple("pointCoor", "x y z") # Coordinates of a point


# Container for the forward kinematic matrices
class fkineMatrix:
    def __init__(self, theta,alpha,d,r):
        self.theta = theta
        self.alpha = alpha
        self.d = d
        self.r = r

    def getMatrix(self):
        matrix = np.array([[math.cos(self.theta),-1*math.sin(self.theta)*math.cos(self.alpha),math.sin(self.theta)*math.sin(self.alpha),self.r*math.cos(self.theta)],[math.sin(self.theta),math.cos(self.theta)*math.cos(self.alpha),-1*math.cos(self.theta)*math.sin(self.alpha),self.r*math.sin(self.theta)],[0,math.sin(self.alpha),math.cos(self.alpha),self.d],[0,0,0,1]])
        return matrix

    def multiplyMatrices(a,b):
        return (a).dot(b)

# Container to hold and seperate data sent by sensor nodes
class sensorData:
    def __init__(self,idnum,anglex,angley,anglez,time,timeRecieve):
      self.idnum = idnum
      self.anglex = anglex
      self.angley = angley
      self.anglez = anglez


      self.time = time
      self.timeRecieve = timeRecieve
   
    def displayData(self):
        s = 'ID:' + str(self.idnum) + ' anglex1:' + str(self.anglex) + ' angley1:' + str(self.angley) + ' anglez1:' + str(self.anglez)  + ' timeSent:' + str(self.time) + ' timeRecieve:' + str(self.timeRecieve) 
        print(s)

    def getData(self):
        return self.idnum, self.anglex, self.angley, self.anglez, self.time, self.timeRecieve

# Thread to recieve and seperate messages sent by each sensor node
def threaded_client(conn,l):

    while True:
        for i in range(NUMBER_OF_NODES):
            data = b''
            while len(data) < 2:
                packet = conn[i].recv(2 - len(data))
                if not packet:
                    data = None
                    break
                data += packet

            raw_msglen = data
            if not raw_msglen:
                continue

            msglen = int(raw_msglen.decode('utf-8'))


            data = conn[i].recv(msglen)
            while len(data) < msglen:
                packet = conn[i].recv(msglen - len(data))
                data += packet


            reply = data.decode('utf-8')
            reply = reply.strip()
            if reply == "":
                continue

            l[i].put(reply)
    conn.close()
    return

# Thread to read and extract data from the messages recieved from sensor nodes
def threaded_reader(l,q):
    startTime = 0
    endTime = 0
    start = 0
    counter = 0
    startTime = time.time()
    while True:
        for i in range(NUMBER_OF_NODES):
            if not l[i].empty():
                while not l[i].empty():
                    temp = l[i].get()
                    lines = temp.split()
                    for x in lines:
                        if x == "start" or x == "quit":
                            continue
                        tokens = x.replace('=',' ').replace('&',' ').split()
                        timeNow = time.time()
                        tempData = sensorData(int(tokens[1]),float(tokens[3]),float(tokens[5]),float(tokens[7]),float(tokens[9]),float(timeNow- startTime))
                        #tempData.displayData()
                        q[int(tokens[1])-1].put(tempData)

# Thread to analyse the data sent by sensor nodes, perform forward kinematic and get arm joint positions
def threaded_forward_kinematic(q,l,k):

    # Calculate end effector position using forward kinematics
    totalDistance = 0
    prevPoint = np.array([0.0,0.0,0.68,1.0])
    prevTime = 0.0
    startShoulderAngle = np.array([0.0,0.0,0.0])
    startElbowAngle = np.array([0.0,0.0,0.0])
    startWristAngle = np.array([0.0,0.0,0.0])
    firstReading = True
    startMeasure = False
    counter = 0
    movement = False
    currentShoulderAngle = np.array([0.0,0.0,0.0])
    currentElbowAngle = np.array([0.0,0.0,0.0])
    currentWristAngle = np.array([0.0,0.0,0.0])
    speed = 0.0
    fileCounter = 1
    fileWrite = False
    fileName = '1.csv'
    f = open(fileName,'w')

    while True:
        if len(q)==NUMBER_OF_NODES:
            while len(q) == NUMBER_OF_NODES:

                shoulderData = q[0].get()
                elbowData = q[1].get()
                newTime = shoulderData.getData()[5]
                #shoulderData.displayData()
                #elbowData.displayData()

                if firstReading == True:
                    startShoulderAngle[0] = shoulderData.getData()[1]
                    startShoulderAngle[1] = shoulderData.getData()[2]
                    startShoulderAngle[2] = shoulderData.getData()[3]
                    startElbowAngle[0] = elbowData.getData()[1]
                    startElbowAngle[1] = elbowData.getData()[2]
                    startElbowAngle[2] = elbowData.getData()[3]
                    startWristAngle[0] = 0
                    startWristAngle[1] = 0
                    startWristAngle[2] = 0
                    firstReading = False

                if not l.empty():
                    temp = l.get()
                    if str(temp) == "reset":
                        print("Position reset")
                        startShoulderAngle[0] = shoulderData.getData()[1]
                        startShoulderAngle[1] = shoulderData.getData()[2]
                        startShoulderAngle[2] = shoulderData.getData()[3]
                        startElbowAngle[0] = elbowData.getData()[1]
                        startElbowAngle[1] = elbowData.getData()[2]
                        startElbowAngle[2] = elbowData.getData()[3]
                        startWristAngle[0] = 0
                        startWristAngle[1] = 0
                        startWristAngle[2] = 0
                        #startMeasure = True
                    if str(temp) == "startExcercise":
                        print("Raise arm to your front")
                        startMeasure = True

                    if str(temp) == "startRecord":
                        print("Recording")
                        if fileCounter >1:
                            fileName = str(fileCounter)+'.csv'
                            f = open(fileName,'w')
                        fileWrite = True
                        fileCounter = fileCounter + 1

                    if str(temp) == "stopRecord":
                        print("Finish Recording")
                        f.close()
                        fileWrite = False

                
                currentShoulderAngle[0] = shoulderData.getData()[1] - startShoulderAngle[0]
                currentShoulderAngle[1] = shoulderData.getData()[2] - startShoulderAngle[1]
                currentShoulderAngle[2] = shoulderData.getData()[3] - startShoulderAngle[2]
                currentElbowAngle[0] = elbowData.getData()[1] - startElbowAngle[0]
                currentElbowAngle[1] = elbowData.getData()[2] - startElbowAngle[1]
                currentElbowAngle[2] = elbowData.getData()[3] - startElbowAngle[2]
                currentWristAngle[0] = 0 - startWristAngle[0]
                currentWristAngle[1] = 0 - startWristAngle[1]
                currentWristAngle[2] = 0 - startWristAngle[2]
                #currentShoulderAngle = 0

                thetas = [currentShoulderAngle[1]*math.pi/180,currentShoulderAngle[2]*math.pi/180,currentShoulderAngle[0]*math.pi/180,(currentElbowAngle[2] - currentShoulderAngle[2])*math.pi/180,(currentWristAngle[1] - currentShoulderAngle[1])*math.pi/180,(currentWristAngle[2] - currentElbowAngle[2]- currentShoulderAngle[2])*math.pi/180,(currentWristAngle[0] - currentShoulderAngle[0])*math.pi/180]
                link1 = fkineMatrix(thetas[0]+math.pi/2,math.pi/2,0,0)
                link2 = fkineMatrix(thetas[1]+math.pi/2,math.pi/2,0,0)
                link3 = fkineMatrix(thetas[2],-1*math.pi/2,0,0.34)
                link4 = fkineMatrix(thetas[3],math.pi/2,0,0.34)
                link5 = fkineMatrix(thetas[4],-1*math.pi/2,0,0)
                link6 = fkineMatrix(thetas[5],-1*math.pi/2,0,0)
                link7 = fkineMatrix(thetas[6],0,0,0)

                elbowTransformationMatrix = fkineMatrix.multiplyMatrices(link1.getMatrix(),fkineMatrix.multiplyMatrices(link2.getMatrix(),link3.getMatrix()))
                newElbowPosition = elbowTransformationMatrix.dot(np.array([0,0,0,1]))
                


                wristTranformationMatrix = fkineMatrix.multiplyMatrices(link1.getMatrix(),fkineMatrix.multiplyMatrices(link2.getMatrix(),fkineMatrix.multiplyMatrices(link3.getMatrix(),fkineMatrix.multiplyMatrices(link4.getMatrix(),fkineMatrix.multiplyMatrices(link5.getMatrix(),fkineMatrix.multiplyMatrices(link6.getMatrix(),link7.getMatrix()))))))
                newWristPosition = wristTranformationMatrix.dot(np.array([0,0,0,1]))
                distance = math.sqrt(math.pow(newWristPosition[0]-prevPoint[0],2)+math.pow(newWristPosition[1]-prevPoint[1],2)+math.pow(newWristPosition[2]-prevPoint[2],2))
                # speed = distance/(newTime - prevTime)

                if fileWrite == True:
                    s = str(newElbowPosition[0]) + ',' + str(newElbowPosition[1]) + ',' + str(newElbowPosition[2]) + ',' + str(newWristPosition[0]) + ',' + str(newWristPosition[1]) + ',' + str(newWristPosition[2]) + ',' + str(shoulderData.getData()[5]) + ',' + str(shoulderData.getData()[5]-shoulderData.getData()[4])+ '\n'
                    f.write(s)

                prevPoint = newWristPosition
                prevTime = newTime

                k[0].put(pointCoor(newElbowPosition[0],newElbowPosition[1],newElbowPosition[2]))
                k[1].put(pointCoor(newWristPosition[0],newWristPosition[1],newWristPosition[2]))