
#sudo chmod 777 /dev/ttyUSB0
#sudo chmod 777 /dev/ttyACM0
import time
from utility import*
from Robotlib_auto import DensoRobotControl
import os
import numpy as np
from HiwinElecGripLib import HiwinGrip
from DeltaTurnTableLib import DeltaServo


# rPosH = [400, -395, 270, 180, 0,-70]
# rPosC = [181, 285, 150, 180, 0,90]
speed = 5
tool =2
fp =60
def robot_operations(robotCut,robotHold,station, eGrip, targetCut, targetHold):
   rHomeH = [350, -315, 270, 180, 0, -90]
   rHomeC = [160, 170, 170, 180, 0, 90]
   eGrip.RunMove(2.5)
   robotHold.GoPosePTP(rHomeH[0], rHomeH[1], rHomeH[2],rHomeH[3], rHomeH[4], rHomeH[5],speed,tool) # home position hold
   robotCut.GoPosePTP(rHomeC[0], rHomeC[1], rHomeC[2],rHomeC[3], rHomeC[4], rHomeC[5],speed,tool) # home position cut 
   station.ServoOn()
   posH = [-targetHold[0], targetHold[1], -targetHold[2], 0, 0, -90]
   posC = [-targetCut[0], targetCut[1], -targetCut[2], 0, 0, -90]

   posH = object2BaseHold(posH)
   posC = object2BaseCut(posC)
   tempH = [posH[0], posH[1],posH[2], 180, 0, targetHold[3]-90]
   tempC = [posC[0], posC[1],posC[2], 180, 0, targetCut[3]-90]

   print("tempH",tempH)
   print("tempC",tempC)
   if(tempH[5] <-180):
      tempH[5] = 360 + tempH[5]
   elif ((tempH[5] >180)):
      tempH[5] = tempH[5] - 360
   if tempH[5] >-45 or tempH[5] <-135:
      if tempH[5] >=0 and tempH[5] <=180:
         angle = tempH[5]-(-90)
      elif tempH[5]<-130 and tempH[5]>=-180:
         angle = tempH[5]-(-90)+ 360
      elif tempH[5]<0 and tempH[5]>-45:
         angle = tempH[5]-(-90)
      ratioA = angle /15
      rotate = int(int(ratioA)*15)
      print("rotate",rotate)
      rPosC = robot2StationCut(tempC, -rotate)
      rPosH = robot2StationHold(tempH, -rotate)
      station.ServoMove(rotate)
   else:
      rPosC = tempC
      rPosH = tempH
   if(tempC[5] <-180):
      tempC[5] = 360 + tempC[5]
   elif ((tempC[5] >180)):
      tempC[5] = tempC[5] - 360
   if(tempC[5] <0):
      tempC[5] = tempC[5] +180
   print("rPosH",rPosH)
   print("rPosC",rPosC)

   rPose = objectNormal([rPosC[0], rPosC[1], rPosC[2],rPosC[3], rPosC[4], rPosC[5]],7)
   robotCut.GoPoseLine(rPose[0], rPose[1], rPose[2]+10,rPose[3], rPose[4], rPose[5],speed,tool,5) # normal position cut 
   # input("enter")
   rPosH = objectNormal(rPosH,-6)
   

   robotHold.GoPosePTP(rPosH[0], rPosH[1], rPosH[2]+10,rPosH[3], rPosH[4], rPosH[5],speed,tool,fp) # normal position
   station.ServoOff()

   rPose[2] = 128
   rPosH[2] = 242
   robotCut.GoPoseLine(rPose[0], rPose[1], rPose[2],rPose[3], rPose[4], rPose[5],speed,tool,95) #  position cut
   robotHold.GoPosePTP(rPosH[0], rPosH[1], rPosH[2],rPosH[3], rPosH[4], rPosH[5],speed,tool,95) # hold position  
   time.sleep(0.5) 
   eGrip.RunForce("L")

   rSlide = objectNormal([rPosC[0], rPosC[1], 128,rPosC[3], rPosC[4], rPosC[5]],-5)
   robotCut.Go2PosePTP(rPose[0], rPose[1], rPose[2]+1,rPose[3], rPose[4]-10, rPose[5],rSlide[0], rSlide[1], rSlide[2]+1,rSlide[3], rSlide[4]-10, rSlide[5],speed,tool) 
   # robotCut.GoPosePTP(rSlide[0], rSlide[1], rSlide[2],rSlide[3], rSlide[4], rSlide[5],speed,tool) #  position cut
   eGrip.RunMove(0)
   robotHold.Go2PosePTP(rPosH[0], rPosH[1], rPosH[2]+20,rPosH[3], rPosH[4], rPosH[5],rHomeH[0], rHomeH[1], rHomeH[2],rHomeH[3], rHomeH[4], rHomeH[5],speed,tool,5) 
   
   robotCut.Go2PosePTP(rPosC[0], rPosC[1], rPosC[2],rPosC[3], rPosC[4], rPosC[5],rHomeC[0], rHomeC[1], rHomeC[2],rHomeC[3], rHomeC[4], rHomeC[5],speed,tool,fp) 
   eGrip.RunMove(2.5)
   station.ServoOff()

   

if __name__ == "__main__":
   station = DeltaServo("/dev/ttyACM1")
   
   eGrip = HiwinGrip("/dev/ttyACM0")
   robotHold = DensoRobotControl("/dev/ttyUSB0")
   robotCut = DensoRobotControl("/dev/ttyUSB1")
   robotHold.SetTimeOut(100, 60000)
   robotCut.SetTimeOut(100, 60000)
   targetCut = [-32.27, 1.22, -459.08, 21.00]
   targetHold = [-30.30, 3.73, -458.76, 41.00]
   input("enter")
   robot_operations(robotCut,robotHold,station, eGrip, targetCut, targetHold)
   
   


   
 




