#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2017 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26
# @version $Id$

from __future__ import absolute_import
from __future__ import print_function

import time as tt
import os
import sys
if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import traci
import optparse
import subprocess
import random
import math
import matplotlib.pyplot as plt
from scipy.linalg import expm
from numpy.linalg import inv
import numpy as np 
import numpy.matlib
from scipy.io import loadmat
from numpy import linalg as LA
from scipy.integrate import solve_ivp
from scipy.integrate import odeint
from scipy.io import savemat
from random import random
# we need to import python modules from the $SUMO_HOME/tools directory
# os.environ['SUMO_HOME'] = "/opt/homebrew/opt/sumo/share/sumo"
# print(os.environ)
# print(os.environ['SUMO_HOME'])
# print(os.path.join(os.environ['SUMO_HOME'],'tools'))
global safe1, lane_change_id, time_all, long_accl_AV_all, Xk_AV, e, y_ang, pos_all, lane_change_flag, vel_all, accel_auto_all, y_k, str_angle, e40, safe_dist
global updateIndx, DxxS, IxxS, IxuS, P, u, KSave, ifLEarnedK20, ifLEarnedK21, ifLEarnedK22, ifLEarnedAll
ifLEarnedAll = 0 
ifLEarnedK20 = 0
ifLEarnedK21 = 0
ifLEarnedK22 = 0
ifLEarnedK20p5 = 0
ifLEarnedK21p5 = 0
ifLEarnedK22p5 = 0

safe_dist = np.array([[0],[0],[0],[0]])
KSave = np.array([[0]])
u = 0
P = 10*np.eye(4);
updateIndx = 1
DxxS = np.zeros((1,16))
IxxS = np.zeros((1,16))
IxuS = np.zeros((1,4))
lane_change_flag = 0
e40 = 0
accel_auto_all = np.array([[]])
long_accl_AV_all = np.array([[]])
Xk_AV = np.array([[],[]])
pos_all = np.array([[],[],[],[],[]])
y_k = np.array([[],[],[],[]])
vel_all = np.array([[],[],[],[],[]])
y_ang = np.array([[],[]])
e = np.array([[],[],[],[]])
str_angle = np.array([[]])
time_all = np.array([[]])
safe1 = 0
lane_change_id = 0


try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")


def generate_routefile():
     # demand per second from different directions
     with open("AB.rou.xml", "w") as routes: 
         # laneChangeModel= "SL2015"
         print("""<routes>
         <vType id="auto" accel="5" decel="4.5" length="5" minGap="1" maxSpeed="20" guiShape="passenger" color="1,0,0"/>
         <vType id="human" accel="5" decel="4.5"  length="5" minGap="1" maxSpeed="20" guiShape="passenger" color="0,0,1"/>
         <vType id="human1" accel="5" decel="4.5"  length="5" minGap="1" maxSpeed="20" guiShape="passenger" color="0,1,0"/>
         <vType id="leader" accel="5" decel="4.5"  length="5" minGap="1" maxSpeed="30" guiShape="passenger" color="0,1,0"/>
         <route id="right" edges="AB BC"/>""", file=routes)
         print('<vehicle id="LT" type="leader" route="right" depart="0" departSpeed="12" departLane="2" departPos="72.5" />', file=routes)
         print('<vehicle id="FT" type="human1" route="right" depart="0" departSpeed="12" departLane="2" departPos="27.5" />', file=routes)
         print('<vehicle id="LC" type="leader" route="right" depart="0" departSpeed="12" departLane="1" departPos="77.5" />', file=routes)
         print('<vehicle id="AV" type="auto" route="right" depart="0" departSpeed="12" departLane="1" departPos="36" />', file=routes)
         print('<vehicle id="FC" type="human1" route="right" depart="0" departSpeed="12" departLane="1" departPos="13.5" />', file=routes)
        
         
         print("</routes>", file=routes)

def learning_K(P, K, DxxS, IxxS, IxuS, Vx):
    
    global KSave
    epsi = 1e-4;
    DxxN = DxxS
    IxxN = IxxS
    IxuN = IxuS
    # print(Vx)
    DxxN = DxxN
    it = 0; 
    POld = np.zeros(4)
    KOld = np.zeros([1,4])
    R = 1
    # Q = np.diag([1, 2, 2000, 3000])
    Q = np.diag([20, 50, 2000, 3000])
    K = K.reshape(1,4)
    
    if Vx == 19:
        KOpt = np.array([[0.999999700139047,	0.0874609257170960,	67.5380080078903,	54.5795865605401]])
    if Vx == 20:
        # KOpt = np.array([[ 4.47213596,   1.44367481, 149.00644403,  53.66556454]])
        KOpt = np.array([[4.47213595499958,	1.44367653964264	,149.006385361161,	53.6655683002131]])
    if Vx == 20.5:
        # KOpt = np.array([[4.472136,     1.48314312, 154.10374218,  53.63200863]])
        KOpt = np.array([[4.47213595499958,	1.46330233558858	,151.564447994438,	53.6489795604414]])
    if Vx == 21:
        # KOpt = np.array([[ 4.47213593,   1.52315368, 159.13125261,  53.59740765]])
        KOpt = np.array([[4.47213595499958,	1.48309144352928	,154.104853021076,	53.6320799423933]])
    if Vx == 21.5:
        # KOpt = np.array([[ 4.47213596,   1.44367481, 149.00644403,  53.66556454]])
        KOpt = np.array([[4.47213595499957,	1.50303672967695	,156.627362185258,	53.6148866975419]])
    if Vx == 22:
        # KOpt = np.array([[4.472136,     1.48314312, 154.10374218,  53.63200863]])
        KOpt = np.array([[4.47213595499959,	1.52313104309674,	159.131758544951	, 53.5974161827215]])
    if Vx == 22.5:
        KOpt = np.array([[4.47213595499963,	1.54336723595436	,161.617845525305,	53.5796839410106]])
    # print('K=',K)
    # while (LA.norm(P-POld) > epsi) and it<100:
    # learning_start_time = traci.simulation.getTime()
    learning_start_time = tt.time()
    while (LA.norm(K-KOld) > epsi) and it<100:
        it = it + 1; 
        POld = P;   
        KOld = K
        Qk = Q + R*np.matmul(K.T,K);
        Theta = np.append(DxxN, -2*np.matmul(IxxN,np.kron(np.eye(4),K.T*R))-2*np.matmul(IxuN,np.kron(np.eye(4),R)), axis=-1)
        Theta = np.array(Theta, dtype='float')
        QkVec = Qk.reshape(16,1)
        Xi = -np.matmul(IxxN,QkVec);
        PK = np.matmul(LA.pinv(Theta),Xi); 
        # PK = LA.inv(Theta.T@Theta)@Theta.T@Xi
        # print(LA.matrix_rank(Theta))
        # print(PK)
        # np.matmul(LA.pinv(Theta),Xi); 
        P = PK[0:16]
        P = P.reshape(4,4)
        P = 0.5*(P+np.transpose(P))
        # print('in loop', 'time', tt.time())
        K = PK[16:20]
        K = K.reshape(1,4)
        KSave = np.append(KSave, LA.norm(np.round(K,3)-np.round(KOpt,3)).reshape(1,1), axis = 0)
    # learning_end_time = traci.simulation.getTime()
    learning_end_time = tt.time()
    print(learning_start_time, learning_end_time)
    print('learning iteration:',it, 'rank Theta=', LA.matrix_rank(Theta))
    print('learning time:', learning_end_time-learning_start_time)
    learningTime = learning_end_time-learning_start_time
    return P, K, learningTime

def getAB(Vx):
    #define system parameters
    m = 1360;
    Iz = 1993;
    lf = 1.45;
    lr = 1.06;
    ls = 0.71;
    Ca = 0.5;
    Re = 0.33;
    I = 3;
    k0 = 460;
    Cf = 1.51*100000;
    Cr = 1.46*100000;
    kx = 1;
    kw = 0.1;
    # print("Vx:", Vx)
    A = np.array([[ 0, 1, 0, 0 ],
                        [ 0, -(1/(m*Vx))*(2*Cf+2*Cr), (2*Cf+2*Cr)/(m), -(2*Cf*lf-2*Cr*lr)/(m*Vx) ],
                        [ 0, 0, 0, 1],
                        [ 0, -(1/(Iz*Vx))*(2*lf*Cf-2*lr*Cr), (2*Cf*lf-2*Cr*lr)/Iz, -(1/(Iz*Vx))*(2*lf*lf*Cf+2*lr*lr*Cr)]])
    B = np.array([[0],[ 2*Cf/m],[ 0], [2*lf*Cf/Iz]])
    
    return A,B
    
    
def model(x,t, K, ifLEarned, Vx, randNum):
    
    A,B = getAB(Vx)
    x = x[0:4]
    if ifLEarned == 0:
        ExpNoiseFreq = loadmat('ExpNoiseFreq1.mat')
        ExpNoiseFreq = ExpNoiseFreq['ExpNoiseFreq']
        #ExpNoise = sum(np.sin(ExpNoiseFreq[0]*10*t))
        ExpNoise = sum(np.sin((ExpNoiseFreq[0]+0/50)*10*t))
        # ExpNoiseFreq = (np.random.rand(1,100)-0.5)
        # print(ExpNoiseFreq)
        # ExpNoise = sum(np.sin(ExpNoiseFreq*t))
        u = -K@x+ExpNoise
        # print('-------uuuu--------')
        # print(u)
        # print('-------uuuu--------')
        # print('-------ifLEarned--------')
        # print(ifLEarned)
        # print('-------ifLEarned--------')
    else:
        u = -K@x
        # print('-------uuuu--------')
        # print(u)
        # print('-------uuuu--------')
        # print('-------ifLEarned--------')
        # print(ifLEarned)
        # print('-------ifLEarned--------')
    # print('---------------')
    # print(u)
    # print('---------------')
    # u = min(0.001, max(-.001, u[0]));

    # print(x)
    dx = A@x.reshape(4,1)+B*u
    dx = dx.reshape(1,4)[0]
    dxx = np.kron(x.T, x.T).tolist();
    dux = np.kron(x.T, u).tolist()
    dx = np.append(dx.tolist(),dxx)
    dx = np.append(dx,dux)
    
    return dx

def run(sampleN=100, epsilon1 = 0.1, flagSave=0):
     """execute the TraCI control loop"""
     step = 0
     N1 = 5000
     dataCollectStart = 1
     distStand = 2
     carW = 0.5
     randNum = random()
     while step < N1:
         
         traci.simulationStep()    

         
         traci.vehicle.setSpeedMode('LT', 0)
         traci.vehicle.setSpeedMode('FT', 0)
         traci.vehicle.setSpeedMode('LC', 0)
         traci.vehicle.setSpeedMode('FC', 0)
         traci.vehicle.setSpeedMode('AV', 0)   

         traci.vehicle.setLaneChangeMode('LT', 0)
         traci.vehicle.setLaneChangeMode('FT', 1)
         traci.vehicle.setLaneChangeMode('LC', 1)
         traci.vehicle.setLaneChangeMode('FC', 1)
         traci.vehicle.setLaneChangeMode('AV', 1)


         Pos_LT_y = traci.vehicle.getPosition('LT')[1]
         Pos_LT_x = traci.vehicle.getPosition('LT')[0]
         
         Pos_FT_y = traci.vehicle.getPosition('FT')[1]
         Pos_FT_x = traci.vehicle.getPosition('FT')[0]
         
         Pos_LC_y = traci.vehicle.getPosition('LC')[1]
         Pos_LC_x = traci.vehicle.getPosition('LC')[0]
         
         Pos_FC_y = traci.vehicle.getPosition('FC')[1]
         Pos_FC_x = traci.vehicle.getPosition('FC')[0]
         
         Pos_AV_y = traci.vehicle.getPosition('AV')[1]
         Pos_AV_x = traci.vehicle.getPosition('AV')[0]
         
         angle_AV = traci.vehicle.getAngle('AV')
         angle_AV = math.radians(angle_AV)

         vel_LT = traci.vehicle.getSpeed('LT')
         vel_FT = traci.vehicle.getSpeed('FT')
         vel_LC = traci.vehicle.getSpeed('LC')
         vel_FC = traci.vehicle.getSpeed('FC')
         vel_AV = traci.vehicle.getSpeed('AV')

         accel_Hum = traci.vehicle.getAcceleration('LT')
         # accel_Hum2 = traci.vehicle.getAcceleration('FT')
         accel_Auto = traci.vehicle.getAcceleration('AV')

         #define system parameters
         m = 1360;
         h = 0.5; # time headway
         dt = 0.01;
         
         global safe1, lane_change_id, time_all, long_accl_AV_all, Xk_AV, e, y_ang, pos_all, lane_change_flag, vel_all, accel_auto_all, y_k, e40, safe_dist
         global DxxS, IxxS, IxuS, P, u, ifLEarnedK20, ifLEarnedK21, ifLEarnedK22, ifLEarnedK20p5, ifLEarnedK21p5, ifLEarnedK22p5, ifLEarnedAll
         if (ifLEarnedK20==1) and (ifLEarnedK21==1) and (ifLEarnedK22==1) and (ifLEarnedK20p5==1) and (ifLEarnedK21p5==1) and (ifLEarnedK22p5==1):
             ifLEarnedAll = 1
             
         if ifLEarnedAll == 0:


                # print('-------K not learned--------')
                
                if (vel_AV<19.8):
                     Vx = 19
                     K = np.array([[0.534522483824849,	0.0208578688649150,	87.8770759497841, 92.4371787404686]])
                  
                if (vel_AV>=20-epsilon1) and (vel_AV<20+epsilon1):
                     Vx = 20
                     K = np.array([[0.534522483824845,	0.0233448161201370,	88.5455628088257, 92.4412563374031]])
                  
                if ((vel_AV)>=20.5-epsilon1) and ((vel_AV)<=20.5+epsilon1):
                     Vx = 20.5
                     K = np.array([[0.534522483824841,	0.0245653386129005,	88.8798926098505, 92.4430785596068]])
                  
                if ((vel_AV)>=21-epsilon1) and  ((vel_AV)<=21+epsilon1):
                     Vx = 21
                     K = np.array([[0.534522483824834,	0.0257723088852428,	89.2142682998254,92.4447698440651]])
                  
                if ((vel_AV)>=21.5-epsilon1) and  ((vel_AV)<=21.5+epsilon1):
                     Vx = 21.5
                     K = np.array([[0.534522483824815,	0.0269668835181444,	89.5486811948735,	92.4463391060863]])
                  
                if ((vel_AV)>=22-epsilon1) and  ((vel_AV)<=22+epsilon1):
                     Vx = 22
                     K = np.array([[0.534522483824906,	0.0281501120727705,	89.8831226795556,92.4477944529148]])
                  
                if ((vel_AV)>=22.5-epsilon1) and  ((vel_AV)<=22.5+epsilon1):
                     Vx = 22.5
                     K = np.array([[0.534522483824853,	0.0293229489436580,	90.2175842070340,92.4491432734948]])
                
                  
         if ifLEarnedAll == 1:
             # print('-------K learned--------')
             if (vel_AV>=20) and (vel_AV<=20.5):
                  Vx = vel_AV
                  K = K20L + (K20p5L-K20L)*( (vel_AV-20)/((20.5-20)) )
                  
             if (vel_AV>=20.5) and  (vel_AV<=21):
                  Vx = vel_AV
                  K = K20p5L + (K21L-K20p5L)*( (vel_AV-20.5)/((21-20.5)) )
                  
             if (vel_AV>=21) and  (vel_AV<=21.5):
                  Vx = vel_AV
                  K = K21L + (K21p5L-K21L)*( (vel_AV-21)/((21.5-21)) )
                  
             if (vel_AV>=21.5) and (vel_AV<=22):
                  Vx = vel_AV
                  K = K21p5L + (K22L-K21p5L)*( (vel_AV-21.5)/((22-21.5)) )
                  
             if (vel_AV>=22) and  (vel_AV<=22.5):
                  Vx = vel_AV
                  K = K22L + (K22p5L-K22L)*( (vel_AV-22)/((22.5-22)) )
                  
             if (vel_AV>22.5):
                  Vx = vel_AV
                  K = K22p5L
                  

                  
             A,B = getAB(vel_AV)

         Kl1 = np.array([[0.316227766016837,	29.3298401558174]])
         # Kl1 = np.array([[0.577350269189626,	39.8379140877438]])
         Kl2 = np.array([[11.1803398874990,	174.744168698121]])
         if step <= 500:
               Kl = Kl1
         else:
               Kl = (1-(step/5000))*Kl1 + (step/5000)*Kl2
               
         # if ifLEarnedK21 == 1:
         #      Kl = np.array([[1.41421356237309,	62.0375764327946]])
              
         # Kl = np.array([[0.577350269189626,	39.8379140877438]])
         
         Al = np.array([[ 0, 1 ],
                            [ 0, 0 ]])
             
         Bl = np.array([[0],[1/m]])
             
         Al = (Al-Bl@Kl)
         Bl = Bl@Kl
             
         Ald = expm(Al*dt)
         Id = np.identity(2)
         Bld = inv(Al)@(Ald-Id)@Bl
            
         if safe1 == 0:

             # K = np.array([[0.141421523183677,	0.0135530294174941,	5.25705973607690	, 2.32303188025642]])
             # K = np.array([[ 0.707106781186154,	0.0311055040090764,	115.740440703782	, 122.316947542560]]) #lqr gain matrix
             # K = np.array([[ 0.999999943651926,	0.0853956021577211,	66.9416128316272	, 54.5756536503324]])
             # K = np.array([[ 0.503907669233854,0.0177293942624530,86.9468175719239,92.4366331064360]]) #not ooptimal
             # K = np.array([[0.99999998,  0.09276839, 68.7828471,  54.57833359]]) #Optimal
             # K = np.array([[1.05431361188091,	0.103593361768276,	68.8794142754593,	53.0842895935385]])
             # K = np.array([[0.3771821551701731, 3.6851141460647185, -46.693107633382745, 19.102323981188697]])
 
             
             # K = np.array([[ 0.149071198499999,	-0.00576333965569731	, 14.3104792338354,	25.6948165766164]])
             # K = np.array([[ 0.534522483824459,	0.0196651592392954,	87.5629827572498	, 92.4350420253785]]) #lqr gain matrix
             
             # Kl = np.array([[100.000000000000,	531.036721894071]])
             # Kl = np.array([[0.577350269189626,	39.8379140877438]])
             # Kl = np.array([[1.00000000000000,	52.6307894677631]])
             # Kl = np.array([[10.0000000000000, 165.227116418583]])
             # Kl = np.array([[0.316227766016837,	29.3298401558174]])
             
             
             
             # Kl_hum= np.array([[1000.00000000000,	1679.28556237467]])
             Kl_hum= np.array([[100.000000000000,	531.036721894071]])
 
    
             e10 = -Pos_AV_y+Pos_LT_y
             e20 = -vel_AV+vel_LT
             e40 = 0;
             d1 = math.dist([Pos_AV_x, Pos_AV_y], [Pos_LT_x, Pos_LT_y])
             d2 = math.dist([Pos_AV_y], [Pos_LT_y])
             # psi_des = math.acos((d2)/(d1))
             psi_des = math.pi/2
             e30 = angle_AV-psi_des
             e0 = np.array([[e10],[e20],[e30],[e40]])
             est_vel = e10
             y_k0 = np.array([[-Pos_AV_y+Pos_LT_y], [0], [angle_AV-math.pi/2], [0]]) # initial value for AV
             y_k0C = np.array([3.299999999999983, 0.734888,  0, 0]) # initial value for AV
             y_k = y_k0
             
             a = np.kron(y_k0C.T, y_k0C.T).tolist()
             b = np.kron(y_k0C.T, 0).tolist()
             y_k0C = np.append(y_k0C.tolist(),a)
             y_k0C = np.append(y_k0C,b)
             
             vel_LT = vel_LT
             Xk_AV0 = np.array([[Pos_AV_x], [vel_AV]]) # initial value
             Xk_FT0 = np.array([[Pos_FT_x], [vel_FT]]) # initial value
             Xk_LC0 = np.array([[Pos_LC_x], [vel_LC]]) # initial value
             Xk_FC0 = np.array([[Pos_FC_x], [vel_FC]]) # initial value
             time = traci.simulation.getTime()

             
             time_all = np.append(time_all,np.array([[time]]))
             Xk_AV = np.append(Xk_AV, Xk_AV0, axis=-1)
             long_accl_AV_all = np.append(long_accl_AV_all, np.array([[accel_Auto]]),  axis=-1)
             accel_auto_all = np.append(accel_auto_all, np.array([[accel_Auto]]),  axis=-1)
             e = np.append(e, e0, axis=-1)
             y_ang = np.append(y_ang, np.array([[-e10+Pos_LT_y],[math.degrees(e30)+90]]), axis=-1)
             str_angle = np.array([[]])

                 # traci.vehicle.setAccel('AV', accel_Hum)
             # safe = 0
             
             # print('---------------')
             # print((Pos_AV_x <= Pos_LT_x-s), Pos_AV_x, Pos_LT_x-s)
             # print((Pos_AV_x >= Pos_FT_x+s), Pos_AV_x, Pos_FT_x+s)
             # print((Pos_AV_x <= Pos_LC_x-s), Pos_AV_x, Pos_LC_x-s)
             # print('---------------')
             pos_all = np.append(pos_all, np.array([[Pos_AV_x],[Pos_LT_x],[Pos_FT_x],[Pos_LC_x],[Pos_FC_x]]), axis=-1)
             vel_all = np.append(vel_all, np.array([[vel_AV],[vel_LT],[vel_FT],[vel_LC],[vel_FC]]), axis=-1)
             ref_vel_FT = vel_LT
             safe_dist = np.append(safe_dist, np.array([[h*vel_LT+distStand+carW],[h*vel_FT+distStand+carW],[h*vel_LC+distStand+carW],[h*vel_FC+distStand+carW]]), axis=-1)
             if (Pos_AV_x <= Pos_LT_x-(h*vel_LT+distStand+carW)) and (Pos_AV_x >= Pos_FT_x+(h*vel_FT+distStand+carW)) and (Pos_AV_x <= Pos_LC_x-(h*vel_LC+distStand+carW)) and (Pos_AV_x >= Pos_FC_x+(h*vel_FC+distStand+carW)) and (traci.simulation.getTime()>3):
                 safe1 = 1
                 lane_change_start_time = traci.simulation.getTime()
                 lane_change_end_time = 0
                 uP = 0
                 e_statesP = e0
                 timeT = 0
                 count = 0
                 ifLEarned = 0
                 e1 = e10
                 Vx0 = Vx
                 lc_timings = np.array([[0],[0]])
                 Vv = Vx
                 Vx0 = Vx
                 print(lc_timings)
                 
                 
             # print(safe1)
             


         if safe1 == 1:  
             time = traci.simulation.getTime()
             time_all = np.append(time_all,np.array([[time]]))
             safe_dist = np.append(safe_dist, np.array([[h*vel_LT],[h*vel_FT],[h*vel_LC],[h*vel_FC]]), axis=-1)
             if (timeT<=30) and (Pos_AV_x <= Pos_LT_x-(h*vel_LT+distStand+carW)) and (Pos_AV_x >= Pos_FT_x+(h*vel_FT+distStand+carW)) and (Pos_AV_x <= Pos_LC_x-(h*vel_LC+distStand+carW)) and (Pos_AV_x >= Pos_FC_x+(h*vel_FC+distStand+carW)):
                Leader_x = Pos_LT_x
                Leader_y = Pos_LT_y
                Leader_vel = vel_LT
                lc = 1
                hypot = math.dist([Pos_AV_x, Pos_AV_y], [Leader_x, Leader_y])
                base = math.dist([Pos_AV_y], [Leader_y])
                vl = 'AB_2'
             if (timeT>30) and (Pos_AV_x <= Pos_LT_x-(h*vel_LT+distStand+carW)) and (Pos_AV_x >= Pos_FT_x+(h*vel_FT+distStand+carW)) and (Pos_AV_x <= Pos_LC_x-(h*vel_LC+distStand+carW)) and (Pos_AV_x >= Pos_FC_x+(h*vel_FC+distStand+carW)):
                if lc == 1:
                    # K= np.array([[ 1.00000009,  0.09270927, 68.7841331,  54.57826932]])
                    # y_k0C = np.array([-Pos_AV_y+Pos_LC_y, -vel_AV+vel_LC, angle_AV-math.pi/2, 0])
                    y_k0C = np.array([-Pos_AV_y+Pos_LC_y, 0, angle_AV-math.pi/2, 0])
                    a = np.kron(y_k0C.T, y_k0C.T).tolist()
                    b = np.kron(y_k0C.T, 0).tolist()
                    y_k0C = np.append(y_k0C.tolist(),a)
                    y_k0C = np.append(y_k0C,b)
                    lc = 0
                    hypot = math.dist([Pos_AV_x, Pos_AV_y], [Leader_x, Leader_y])
                    base = -math.dist([Pos_AV_y], [Leader_y])
                    lane_change_start_time = traci.simulation.getTime()
                    lane_change_end_time = 0
                    lane_change_flag = 0
                    vl = 'AB_1'
                    

                Leader_x = Pos_LC_x
                Leader_y = Pos_LC_y
                Leader_vel = vel_LC
                
                # Xk_AV0 = np.array([[Pos_AV_x], [vel_AV]])
                
             # Leader_x = Pos_LT_x
             # Leader_y = Pos_LT_y
            
             # psi_des = math.acos((base)/(hypot))
             psi_des = math.pi/2
             
             # lqr for human vehicle
             
             # print('lane_change_start_time=', lane_change_start_time)
             # print('Time=', timeT)
             timeT = timeT+dt
             t = np.linspace(time-dt,time,11)
             # print("round vel_AV", np.round(vel_AV), "vel_AV", (vel_AV))
             # print("Vx:", Vx)
             # print('-------K--------')
             # print(K)
             # print('---------------')
             # print('-------ifLEarned--------')
             # print(ifLEarned)
             # print('---------------')
             y_kC = odeint(model,y_k0C,t,atol=1e-10,rtol=1e-10,args=(K,ifLEarned, vel_AV,randNum ))

             # y_kC = solve_ivp(model, t, y_k0C, method = 'RK45')
            
             y_kC0 = y_kC[0,:]
             y_kC1 = y_kC[-1,:]
             
             
                     
             e1 =  y_kC1[0]
             e2 =  y_kC1[1]
             e3 =  y_kC1[2]
             e4 =  y_kC1[3]
             e10 = e1
             e20 = e2
             e40 = e4
             e30 = e3

             e0 = np.array([[e1],[e2],[e3],[e4]])
             # print(Vx)
             if (Vx != Vx0) and (ifLEarnedAll == 0):
                 print('in',"Vx:", Vx)
                 Vv = Vx
                 DxxS = np.zeros((1,16))
                 IxxS = np.zeros((1,16))
                 IxuS = np.zeros((1,4))
                 ifLEarned=0
            
             #-----Data Collection and learning-----#
             if (Vx == Vv) and (ifLEarned==0) and (ifLEarnedAll == 0) and (vel_AV>=Vx-epsilon1) and (vel_AV<Vx+epsilon1):
             # if (Vx == Vv) and (ifLEarned==0) and (ifLEarnedAll == 0) and (vel_AV>=Vx-epsilon1) and (vel_AV<Vx+epsilon1):
                 if dataCollectStart==1:
                     dataCollectStartTime = traci.simulation.getTime()
                     dataCollectStart = 0
                 y_kC0 = y_kC[0,:]
                 y_kC1 = y_kC[-1,:]
                 e_states = y_kC1[0:4]
                 e_states = np.transpose(e_states)
                 e_statesP = y_kC0[0:4]
                 b0 = np.kron(e_states,e_states)-np.kron(e_statesP,e_statesP)
                 b0 = b0.reshape((1, 16))
                 DxxS = np.append(DxxS, b0, axis=0)
        
                 b1 = y_kC1[4:20]-y_kC0[4:20]
                 b1 = b1.reshape((1, 16))
                 IxxS = np.append(IxxS, b1, axis=0)
         
                 b2 = y_kC1[20:24]-y_kC0[20:24]
                 b2 = b2.reshape((1, 4))
                 IxuS = np.append(IxuS, b2, axis=0)
         
                 e_statesP = np.transpose(e_states)
                 #print("count:", count, "velAV:", vel_AV, "Vx:", Vx)
                 count = count + 1
                 # print(count)
                 # uP = u
                 # P,K = learning_K(P, K, DxxS, IxxS, IxuS)
                 if (count == sampleN):
                     if flagSave == 1:
                         savemat("dataForLearningInMATLABVx"+str(Vx)+"Epsilon1_"+str(epsilon1)+".mat", {"Dxx":DxxS, "XX":IxxS, "XU":IxuS, "KSave":KSave})
                     dataCollectEndTime = traci.simulation.getTime()
                     print("Vx:", Vx)
                     print("count:", count)
                     print("dataCollectionTime:", dataCollectEndTime-dataCollectStartTime)
                     # learning_start_time = tt.time()
                     P, K ,learningTime = learning_K(P, K, DxxS, IxxS, IxuS, Vx)
                     # learning_end_time = tt.time()
                     control_update_time = traci.simulation.getTime()
                     # LearningTime = np.array()
                     print("K=",K)
                     # print("learning time:", learning_end_time-learning_start_time)
                     count = 0
                     dataCollectStart = 1
                     ifLEarned=1
                 if (Vx == 20) and (ifLEarned==1):
                     K20L = K
                     ifLEarnedK20 = 1
                     learningTimeK20 = learningTime
                 if (Vx == 20.5) and (ifLEarned==1):
                     K20p5L = K
                     ifLEarnedK20p5 = 1
                     learningTimeK20p5 = learningTime
                 if (Vx == 21) and (ifLEarned==1):
                     K21L = K
                     ifLEarnedK21 = 1
                     learningTimeK21 = learningTime
                 if (Vx == 21.5) and (ifLEarned==1):
                     K21p5L = K
                     ifLEarnedK21p5 = 1
                     learningTimeK21p5 = learningTime
                 if (Vx == 22) and (ifLEarned==1):
                     K22L = K
                     ifLEarnedK22 = 1
                     learningTimeK22 = learningTime
                 if (Vx == 22.5) and (ifLEarned==1):
                     K22p5L = K
                     ifLEarnedK22p5 = 1
                     learningTimeK22p5 = learningTime
             Vx0 = Vx   
             
             #---------------------#
             y_k0C = y_kC1
             # longitudinal dynamics of autonomous vehicle
             if ifLEarnedK21 == 1:
                 longVelRef = 22
             else:
                 longVelRef = Leader_vel
                 
             r = np.array([[Leader_x-(h*vel_LT+distStand+carW)],[Leader_vel]])
             # ul = Kl@(-Xk_AV0+r)  
             # Xk_AV1 = Xk_AV0 + Ald@Xk_AV0*dt + Bld@ul*dt
             ul = Kl@(-Xk_AV0+r)
             Xk_AV1 = Ald@Xk_AV0 + Bld@r
             # Xk_AV1[1] = max(19, Vx)
             Xk_AV = np.append(Xk_AV, Xk_AV1, axis=-1)
             Xk_AV0 = Xk_AV1
             long_accl_AV = ul/m
             
             # ul = Kl@(-Xk_AV0+r)
             # Xk_AV1 = Ald@Xk_AV0 + Bld@u
             
             
             # Pos_AV_x = traci.vehicle.getPosition('AV')[0]
             
             ang = 90+math.degrees(e3)
             # print("Y Pos=", -e1+Pos_LT_y, "angle=",ang, "u=",u)
             # print('ang=', ang)
             Lid = traci.vehicle.getLaneID('AV')

             
             
                 
             if (Lid==vl) and (lane_change_flag == 0):
             # if (abs(Leader_y-Pos_AV_y)<=0.5) and (lane_change_flag == 0):
                 lane_change_flag = 1
                 lane_change_end_time = traci.simulation.getTime()
                 lane_change_y_dist = -e1+Pos_LT_y-Pos_LT_y
                 ref_vel_FT = vel_LT
                 lc_timings = np.append(lc_timings,np.array([[lane_change_start_time],[lane_change_end_time]]), axis = -1)
                 print('lane change time = ', lane_change_end_time-lane_change_start_time)
                 # Kl = np.array([[100.000000000000,	531.036721894071]])

             traci.vehicle.moveToXY(vehID='AV',edgeID='AB', lane=2, x=Xk_AV1[0], y=-e1+Leader_y, angle=ang, keepRoute=2, matchThreshold=100000)
             traci.vehicle.setSpeed('AV', Xk_AV1[1])
             if long_accl_AV>0:
                   traci.vehicle.setAccel('AV', long_accl_AV)
             else:
                   traci.vehicle.setDecel('AV', -long_accl_AV)
             
              # traci.vehicle.setAccel('LC', accel_Hum)
             traci.vehicle.setSpeed('LC', vel_LT)
             
             if lane_change_flag == 1:
                 a=1
                 # if (vel_AV<19):
                 #     Vx = 19
             
                 #    # K = np.array([[ 0.534522483824849,	0.0208578688649150,	87.8770759497841	, 92.4371787404686]])
             
                 #     K = np.array([[ 1.00000000527919,	0.0877230026686675,	67.5314723997130	, 54.5767696071454]])
             
                 # if (vel_AV>=19) and (vel_AV<=20):
                 #     Vx = 19
             
                 #     # K19 = np.array([[ 0.534522483824849,	0.0208578688649150,	87.8770759497841	, 92.4371787404686]])
                 #     # K20 = np.array([[ 0.534522483824845,	0.0233448161201370,	88.5455628088257	, 92.4412563374031]])
                    
                 #     K19 = np.array([[ 1.00000000527919,	0.0877230026686675,	67.5314723997130	, 54.5767696071454]])
                 #     K20 = np.array([[ 1.00000000301250,	0.0927505384441361,	68.7832225993298, 	54.5784173500620]])
             
                 #     weightParam = vel_AV-20
                 #     K = (weightParam)*K19 + (1-weightParam)*K20
                     
                 # if (vel_AV>=20) and (vel_AV<=21):
                 #     Vx = 20
             
                 #    # K20 = np.array([[ 0.534522483824845,	0.0233448161201370,	88.5455628088257	, 92.4412563374031]])
                 #    # K21 = np.array([[ 0.534522483824834,	0.0257723088852428,	89.2142682998254	, 92.4447698440651]])
                    
                 #     K20 = np.array([[ 1.00000000301250,	0.0927505384441361,	68.7832225993298, 	54.5784173500620]])
                 #     K21 = np.array([[ 1.00000000012269,	0.0977267294501871,	70.0336808997636	, 54.5793384774861]])
             
                 #     weightParam = vel_AV-21
                 #     K = (weightParam)*K20 + (1-weightParam)*K21
                 # if (vel_AV>21):
                 #     Vx = 21
             
                 #     # K = np.array([[ 0.534522483824834,	0.0257723088852428,	89.2142682998254	, 92.4447698440651]])
             
                 #     K = np.array([[ 1.00000000012269,	0.0977267294501871,	70.0336808997636	, 54.5793384774861]])
         
                 
                 
             else:
                  Xk_FT0 = np.array([[Pos_FT_x], [vel_FT]]) # initial value
                  Xk_LC0 = np.array([[Pos_LC_x], [vel_LC]]) # initial value
                  Xk_FC0 = np.array([[Pos_FC_x], [vel_FC]]) # initial value
             
             # collect data
             pp = Pos_AV_y
             Pos_LT_x = traci.vehicle.getPosition('LT')[0]
             Pos_LT_y = traci.vehicle.getPosition('LT')[1]
             Pos_FT_x = traci.vehicle.getPosition('FT')[0]
             Pos_LC_x = traci.vehicle.getPosition('LC')[0]
             Pos_FC_x = traci.vehicle.getPosition('FC')[0]
             Pos_AV_x = traci.vehicle.getPosition('AV')[0]
             Pos_AV_y = traci.vehicle.getPosition('AV')[1]
             
             vel_LT = traci.vehicle.getSpeed('LT')
             vel_FT = traci.vehicle.getSpeed('FT')
             vel_LC = traci.vehicle.getSpeed('LC')
             vel_FC = traci.vehicle.getSpeed('FC')
             vel_AV = traci.vehicle.getSpeed('AV')
             
             accel_Auto = traci.vehicle.getAcceleration('AV')
             
             angle_AV = traci.vehicle.getAngle('AV')
             angle_AV = math.radians(angle_AV)
             d1 = math.dist([Pos_AV_x, Pos_AV_y], [Pos_LT_x, Pos_LT_y])
             d2 = math.dist([Pos_AV_y], [Pos_LT_y])
             # psi_des = math.acos((d2)/(d1))
             psi_des = math.pi/2
             # print("actual values:", -Pos_AV_y+Pos_LT_y, angle_AV-psi_des, "simulated values:",y_kC1[0],y_kC1[2])
             
               
             # y_k0C = np.array([-Pos_AV_y+Leader_y, e2, e3, e4])
             # a = np.kron(y_k0C.T, y_k0C.T).tolist()
             # b = np.kron(y_k0C.T, 0).tolist()
             # y_k0C = np.append(y_k0C.tolist(),a)
             # y_k0C = np.append(y_k0C,b)
             
             pos_all = np.append(pos_all, np.array([[Pos_AV_x],[Pos_LT_x],[Pos_FT_x],[Pos_LC_x],[Pos_FC_x]]), axis=-1)
             vel_all = np.append(vel_all, np.array([[vel_AV],[vel_LT],[vel_FT],[vel_LC],[vel_FC]]), axis=-1)
             e = np.append(e, np.array([[e1],[e2],[e3],[e4]]), axis=-1)
             y_ang = np.append(y_ang, np.array([[Pos_AV_y],[angle_AV]]), axis=-1)
             long_accl_AV_all = np.append(long_accl_AV_all,long_accl_AV,  axis=-1)
             accel_auto_all = np.append(accel_auto_all, np.array([[accel_Auto]]),  axis=-1)
             # str_angle = np.append(str_angle, u, axis=-1)
             
 
             
         step += 1
     
     # print(Xk_AV)
     # plt.plot(time_all, Xk_AV[1,:])
     # plt.plot(laneIndx)
     # plt.show()
     # print(long_accl_AV_all)
     # plt.plot(time_all, np.transpose(long_accl_AV_all))
     # plt.show()
     print(lc_timings)
     
     # savemat("AV_all_data_Learning_Lane_Changing_Data.mat", {"Xk_AV":Xk_AV,"e":e, "y_ang":y_ang, "long_accl_AV_all":long_accl_AV_all, \
     #                                      "time_all":time_all, "vel_all":vel_all, "accel_auto_all":accel_auto_all, \
     #                                      "pos_all":pos_all, "lane_change_start_time":lane_change_start_time,\
     #                                      "lane_change_end_time":lane_change_end_time, "lane_change_y_dist":lane_change_y_dist,\
     #                                      "lc_timings":lc_timings, "KSave":KSave, "control_update_time":control_update_time,\
     #                                      "safe_dist":safe_dist})
     
     # savemat("Gain_Scheduling_Learning_Data_K20_K21.mat", {"Xk_AV":Xk_AV,"e":e, "y_ang":y_ang, "long_accl_AV_all":long_accl_AV_all, \
     #                                             "time_all":time_all, "vel_all":vel_all, "accel_auto_all":accel_auto_all, \
     #                                             "pos_all":pos_all, "lane_change_start_time":lane_change_start_time,\
     #                                             "lane_change_end_time":lane_change_end_time, "lane_change_y_dist":lane_change_y_dist,\
     #                                             "lc_timings":lc_timings, "KSave":KSave, "safe_dist":safe_dist, \
     #                                             "control_update_time":control_update_time, "learningTimeK20":learningTimeK20,\
     #                                             "learningTimeK21":learningTimeK21})
     
     # savemat("Gain_Scheduling_0p5_Learning_Data_K20_K21_K22_New_Q_newSafety.mat", {"Xk_AV":Xk_AV,"e":e, "y_ang":y_ang, "long_accl_AV_all":long_accl_AV_all, \
     #                                                    "time_all":time_all, "vel_all":vel_all, "accel_auto_all":accel_auto_all, \
     #                                                    "pos_all":pos_all, "lane_change_start_time":lane_change_start_time,\
     #                                                    "lane_change_end_time":lane_change_end_time, "lane_change_y_dist":lane_change_y_dist,\
     #                                                    "lc_timings":lc_timings, "KSave":KSave, "safe_dist":safe_dist, \
     #                                                    "control_update_time":control_update_time, "learningTimeK20":learningTimeK20,
     #                                                    "learningTimeK21":learningTimeK21, "learningTimeK22":learningTimeK22})
     
     
     # savemat("ADP_Sumo_Data.mat", {"IxxS":IxxS,"DxxS":DxxS,"IxuS":IxuS})
     # savemat("AV_safety_data1.mat", {"dist_all":pos_all, "lane_change_start_time":lane_change_start_time,\
                                     # "lane_change_end_time":lane_change_end_time, "lane_change_y_dist":lane_change_y_dist})
     # np.save('auto_longitudinal_data', Xk_AV)
     # print(laneIndx)
     traci.close()
     sys.stdout.flush()
     return learningTime
     


def get_options():
     optParser = optparse.OptionParser()
     optParser.add_option("--nogui", action="store_true",
                          default=False, help="run the commandline version of sumo")
     options, args = optParser.parse_args()
     return options


# this is the main entry point of this script
# if __name__ == "__main__":
#      options = get_options()

#      # this script has been called from the command line. It will start sumo as a
#      # server, then connect and run
#      if options.nogui:
#          sumoBinary = checkBinary('sumo')
#      else:
#          sumoBinary = checkBinary('sumo-gui')

#      # first, generate the route file for this simulation
#      generate_routefile()

#      # this is the normal way of using traci. sumo is started as a
#      # subprocess and then the python script connects and runs
#      traci.start([sumoBinary, "-c", "ABC.sumocfg"])
#      run()
     
     
# this is the main entry point of this script
if __name__ == "__main__":
     options = get_options()

     # this script has been called from the command line. It will start sumo as a
     # server, then connect and run
     if options.nogui:
         sumoBinary = checkBinary('sumo')
     else:
         sumoBinary = checkBinary('sumo-gui')

     # first, generate the route file for this simulation
     generate_routefile()

     # this is the normal way of using traci. sumo is started as a
     # subprocess and then the python script connects and runs
     LearningTimesAll = np.array([[],[],[],[],[],[]])
     for i in range(1):
          if i>0:
             ifLEarnedAll = 0 
             ifLEarnedK20 = 0
             ifLEarnedK21 = 0
             ifLEarnedK22 = 0
             ifLEarnedK20p5 = 0
             ifLEarnedK21p5 = 0
             ifLEarnedK22p5 = 0
            
             safe_dist = np.array([[0],[0],[0],[0]])
             KSave = np.array([[0]])
             u = 0
             P = 10*np.eye(4);
             updateIndx = 1
             DxxS = np.zeros((1,16))
             IxxS = np.zeros((1,16))
             IxuS = np.zeros((1,4))
             lane_change_flag = 0
             e40 = 0
             accel_auto_all = np.array([[]])
             long_accl_AV_all = np.array([[]])
             Xk_AV = np.array([[],[]])
             pos_all = np.array([[],[],[],[],[]])
             y_k = np.array([[],[],[],[]])
             vel_all = np.array([[],[],[],[],[]])
             y_ang = np.array([[],[]])
             e = np.array([[],[],[],[]])
             str_angle = np.array([[]])
             time_all = np.array([[]])
             safe1 = 0
             lane_change_id = 0
          traci.start([sumoBinary, "-c", "ABC.sumocfg"])
          LearningTimes = run()
          LearningTimesAll = np.append(LearningTimesAll, LearningTimes, axis=-1)
          print("LearningTimes:", LearningTimes, "iterations:", i)
          from scipy.io import savemat
          savemat("learning_Times.mat", {"LearningTimesAll":LearningTimesAll})
     # for i in range(500):
     #      if i>0:
     #         ifLEarnedAll = 0 
     #         ifLEarnedK20 = 0
     #         ifLEarnedK21 = 0
     #         ifLEarnedK22 = 0
     #         ifLEarnedK20p5 = 0
     #         ifLEarnedK21p5 = 0
     #         ifLEarnedK22p5 = 0
            
     #         safe_dist = np.array([[0],[0],[0],[0]])
     #         KSave = np.array([[0]])
     #         u = 0
     #         P = 10*np.eye(4);
     #         updateIndx = 1
     #         DxxS = np.zeros((1,16))
     #         IxxS = np.zeros((1,16))
     #         IxuS = np.zeros((1,4))
     #         lane_change_flag = 0
     #         e40 = 0
     #         accel_auto_all = np.array([[]])
     #         long_accl_AV_all = np.array([[]])
     #         Xk_AV = np.array([[],[]])
     #         pos_all = np.array([[],[],[],[],[]])
     #         y_k = np.array([[],[],[],[]])
     #         vel_all = np.array([[],[],[],[],[]])
     #         y_ang = np.array([[],[]])
     #         e = np.array([[],[],[],[]])
     #         str_angle = np.array([[]])
     #         time_all = np.array([[]])
     #         safe1 = 0
     #         lane_change_id = 0
     #      traci.start([sumoBinary, "-c", "ABC.sumocfg"])
     #      LearningTimes = run()
     #      LearningTimesAll = np.append(LearningTimesAll, LearningTimes, axis=-1)
     #      print("LearningTimes:", LearningTimes, "iterations:", i)
     #      from scipy.io import savemat
     #      savemat("learning_Times.mat", {"LearningTimesAll":LearningTimesAll})

