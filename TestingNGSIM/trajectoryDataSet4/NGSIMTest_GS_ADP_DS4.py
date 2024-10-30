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
import pandas as pd
# we need to import python modules from the $SUMO_HOME/tools directory
# os.environ['SUMO_HOME'] = "/opt/homebrew/opt/sumo/share/sumo"
# print(os.environ)
# print(os.environ['SUMO_HOME'])
# print(os.path.join(os.environ['SUMO_HOME'],'tools'))

global safe1, lane_change_id, time_all, long_accl_AV_all, Xk_AV, e, y_ang, pos_all, lane_change_flag, vel_all, accel_auto_all, y_k, str_angle, e40, safe_dist, pos_y_all, time_TTC_FT_AV_LT, time_TTC_LC_AV
global updateIndx, DxxS, IxxS, IxuS, P, u, change_gain, accelFT, if_des_lane, init1, initateLaneChange, initateLaneAbort, randNum, e20, agg_gain_time2, ttc_AV_FT_all, ttc_LT_AV_all, ttc_LC_AV_all, St

# Pos_LT_y0 = 0
# Pos_AV_y0 = 0
# Pos_LC_y0 = 0
ttc_AV_FT_all = np.array([])
ttc_LT_AV_all = np.array([])
ttc_LC_AV_all = np.array([])
e20 = 0
agg_gain_time2 = 0
# randNum = np.random.uniform(0,1,1)[0]
randNum = 0.4
initateLaneChange = 1
initateLaneAbort = 1
init1 = 0
accelFT = 0
change_gain = 0
safe_dist = np.array([[0],[0],[0],[0]])
u = 0
P = 10*np.eye(4);
updateIndx = 1
DxxS = np.zeros((1,16))
IxxS = np.zeros((1,16))
IxuS = np.zeros((1,4))
lane_change_flag = 0
e40 = 0
accel_auto_all = np.array([])
long_accl_AV_all = np.array([])
Xk_AV = np.array([0,0])
pos_all = np.array([[],[],[],[],[]])
St = np.array([[],[],[],[]])
pos_y_all = np.array([[],[],[],[],[]])
y_k = np.array([[],[],[],[]])
vel_all = np.array([[],[],[],[],[]])
y_ang = np.array([[],[]])
e = np.array([[],[],[],[]])
str_angle = np.array([[]])
time_all = np.array([[]])
safe1 = 0
lane_change_id = 0
if_des_lane = 0
time_TTC_FT_AV_LT = np.array([])
time_TTC_LC_AV = np.array([])


try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

# Directory where CSV files are stored
directory = 'D:/Sayan Github/gain-scheduling-simulations/TestingNGSIM/trajectoryDataSet4'

# Dictionary to hold data from each CSV file
trajectories = {}


# Loop through each file in the directory
for filename in os.listdir(directory):
    if filename.endswith(".csv"):
        file_path = os.path.join(directory, filename)
        # Read the CSV file
        df = pd.read_csv(file_path)
        # Store the dataframe in a dictionary with the filename as the key
        trajectories[filename] = df


# extract position and velocity for each follower, leader and subject vehicle
FC = trajectories['veh_r.csv']
FCPos = FC[['y', 'x']]  # Note: Local_Y in data is Local_X here, and vice-versa
FCVel = FC['v']  
FCAccl = FC['a']  

LC = trajectories['veh_f.csv']
LCPos = LC[['y', 'x']]  # Note: Local_Y in data is Local_X here, and vice-versa
LCVel = LC['v']  
LCAccl = LC['a'] 

LT = trajectories['veh_ft.csv']
LTPos = LT[['y', 'x']]  # Note: Local_Y in data is Local_X here, and vice-versa
LTVel = LT['v'] 
LTAccl = LT['a']

FT = trajectories['veh_rt.csv']
FTPos = FT[['y', 'x']]  # Note: Local_Y in data is Local_X here, and vice-versa
FTVel = FT['v']  
FTAccl = FT['a']

AV = trajectories['veh_s.csv']
AVPos = AV[['y', 'x']]  # Note: Local_Y in data is Local_X here, and vice-versa
AVVel = AV['v']  
AVAccl = AV['a']


# Use this data to set vehicle positions and speeds in your SUMO simulation


def generate_routefile():
     # demand per second from different directions
     with open("NGSIMTest.rou.xml", "w") as routes: 
         # laneChangeModel= "SL2015"
         print("""<routes>
         <vType id="auto" accel="5" decel="4.5" length="5" minGap="1" maxSpeed="20" guiShape="passenger" color="1,0,0"/>
         <vType id="human" accel="5" decel="4.5"  length="5" minGap="1" maxSpeed="20" guiShape="passenger" color="0,0,1"/>
         <route id="right" edges="AB"/>""", file=routes)
           
         print('<vehicle id="AV" type="auto" route="right" depart="0" departSpeed="6.931152" departLane="0" departPos="133.680708" />', file=routes) # AV
         print('<vehicle id="FC" type="human" route="right" depart="0" departSpeed="8.918448" departLane="0" departPos="105.930192" />', file=routes) # FC (1)
         print('<vehicle id="FT" type="human" route="right" depart="0" departSpeed="4.730496" departLane="0" departPos="102.5374632" />', file=routes) # FT (2)
         print('<vehicle id="LC" type="human" route="right" depart="0" departSpeed="6.52272" departLane="0" departPos="144.5413416" />', file=routes) # LC (3)
         print('<vehicle id="LT" type="human" route="right" depart="0" departSpeed="7.900416" departLane="0" departPos="146.7685152" />', file=routes) # LT (4)
    
         
         
         print("</routes>", file=routes)
         
         
def ChooseGain(vel_AV):
    
         ## The following gains are obtained for Q = diag([20 50 2000 3000]), R = 1
         K5p5L = np.array([[4.1979,   1.4133,   66.0491,   53.3715]])
         K6p3L = np.array([[4.2258,    1.2460,   71.1618,   53.5973]])
         K7p1L = np.array([[4.3485,    1.2945,   75.7725,   53.5941]])
         K7p9L = np.array([[4.5027,    1.1029,   82.3021,   53.8077]])
         K8p7L = np.array([[4.2135,    1.1283,   85.2675,   53.8252]])
         K9p5L = np.array([[4.6256,    1.1975,   90.9199,   53.7782]])
         
                 
             
         space = 0.8
         if (vel_AV<5.5):
             print("vel_AV", vel_AV, ", in: < 5.5")
             K = K5p5L
             
         if (vel_AV>5.5) and (vel_AV<=6.3):
                    print("vel_AV", vel_AV, ", in: 5.5 to 6.3")
                    K = K5p5L + (K6p3L-K5p5L)*( (vel_AV-5.5)/((space)) )
                  
         if (vel_AV>6.3) and  (vel_AV<=7.1):
                    print("vel_AV", vel_AV, ", in: 6.3 to 7.1")
                    K = K6p3L + (K7p1L-K6p3L)*( (vel_AV-6.3)/((space)) )
                  
         if (vel_AV>7.1) and  (vel_AV<=7.9):
                    print("vel_AV", vel_AV, ", in: 7.1 to 7.9")
                    K = K7p1L + (K7p9L-K7p1L)*( (vel_AV-7.1)/((space)) )
                  
         if (vel_AV>7.9) and  (vel_AV<=8.7):
                    print("vel_AV", vel_AV, ", in: 7.9 to 8.7")
                    K = K7p9L + (K8p7L-K7p9L)*( (vel_AV-7.9)/((space)) )
                  
         if (vel_AV>8.7) and (vel_AV<=9.5):
                    print("vel_AV", vel_AV, ", in: 8.7 to 9.5")
                    K = K8p7L + (K8p7L-K9p5L)*( (vel_AV-8.7)/((space)) )
                  
         if (vel_AV>9.5):
             print("vel_AV", vel_AV, ", in: > 9.5")
             K = K9p5L
                  
         return K
     
def model_longi(x,t,v, Kl, OutReg):
    m = 1360
    
    if OutReg == 1:
        ## ------- output regulation -------- ##
        # Kl = np.array([[0.0715262765604084,	0.384583524450610]])
        # Ll = np.array([[0.0714426181991808,	0.390262863029821]])
        Kl = np.array([[0.316235341317086,	0.855854112990615]])
        Ll = np.array([[0.316235341317086,	0.855854112990615]])
        u = -Kl@x.reshape(2,1)+Ll@v.reshape(2,1)
        A = np.array([[ 0, 1 ],
                    [ 0, 0 ]])       
        B = np.array([[0],[1]])
        dx = A@x.reshape(2,1)+B*u
        ## ----------------------------------- ##
    else:
        ## ------- No output regulation -------- ##
        u = -Kl@x.reshape(2,1)+Kl@v.reshape(2,1)
        A = np.array([[ 0, 1 ],
                        [ 0, 0 ]])       
        # B = np.array([[0],[1/m]])
        B = np.array([[0],[1/m]])
        dx = A@x.reshape(2,1)+B*u
        ## ----------------------------------- ##
    
    dx = dx.reshape(1,2)[0]
    return dx

def model(x,t, K, ifLEarned, Vx):
    
    A,B = getAB(Vx)
    x = x[0:4]
    if ifLEarned == 0:
        ExpNoiseFreq = loadmat('ExpNoiseFreq1.mat')
        ExpNoiseFreq = ExpNoiseFreq['ExpNoiseFreq']
        ExpNoise = sum(np.sin(ExpNoiseFreq[0]*100*t))
        # ExpNoiseFreq = (np.random.rand(1,100)-0.5)
        # print(ExpNoiseFreq)
        # ExpNoise = sum(np.sin(ExpNoiseFreq*t))
        u = -K@x+ExpNoise
    else:
        u = -K@x
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
    # print(dx)
    return dx

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


# def run(h, AV_constant_vel, UseOutReg, NonCop):
def run(h=0.65, AV_constant_vel=0):
     """execute the TraCI control loop"""
     
     
     step = 0
     N1 = 1990
     traci.simulationStep()    
     sim_start_time = traci.simulation.getTime()
     accLT = 0
     LC_init = 0
     TTA = 0
     aaaa = 0
     aaa = 0
     initateLaneFollow = 1
     OutReg = 0
     force_accel = np.array([])
     steer_angle_all = np.array([[]])
     decideAbort = 0
     positions_vehicles = np.array([255, 210, 260, 16, 40])
     FT_aggressive_count = 0
     distStand = 2
     carW = 0.5
     dt = 0.01;
     m = 1360; # mass of vehicle
     while (step < N1):        
         
         traci.simulationStep()   
         
         # subject vehicle
         traci.vehicle.moveToXY('AV', edgeID='', lane=0, x=AVPos.loc[step, 'y'], y=AVPos.loc[step, 'x']) # NOTE: X, Y positions are flipped to match orientation of the network
         traci.vehicle.setSpeed('AV', AVVel[step])
         # if  AVAccl[step]>0:
         #          traci.vehicle.setAccel('0', AVAccl[step])
         # else:
         #          traci.vehicle.setDecel('0', -AVAccl[step])
                  
         
         # followers
         traci.vehicle.moveToXY('FC', edgeID='', lane=0, x=FCPos.loc[step, 'y'], y=FCPos.loc[step, 'x']) # NOTE: X, Y positions are flipped to match orientation of the network
         traci.vehicle.setSpeed('FC', FCVel[step])
         
                     
         traci.vehicle.moveToXY('FT', edgeID='', lane=0, x=FTPos.loc[step, 'y'], y=FTPos.loc[step, 'x']) # NOTE: X, Y positions are flipped to match orientation of the network
         traci.vehicle.setSpeed('FT', FTVel[step])      
     
         
         # leaders
         traci.vehicle.moveToXY('LC', edgeID='', lane=0, x=LCPos.loc[step, 'y'], y=LCPos.loc[step, 'x']) # NOTE: X, Y positions are flipped to match orientation of the network
         traci.vehicle.setSpeed('LC', LCVel[step])
         
         
         traci.vehicle.moveToXY('LT', edgeID='', lane=0, x=LTPos.loc[step, 'y'], y=LTPos.loc[step, 'x']) # NOTE: X, Y positions are flipped to match orientation of the network
         traci.vehicle.setSpeed('LT', LTVel[step])
         
         
         
         
         
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
         
         accel_Hum = traci.vehicle.getAcceleration('LT')
         # accel_Hum2 = traci.vehicle.getAcceleration('FT')
         accel_Auto = traci.vehicle.getAcceleration('AV')
         
         vel_LT = traci.vehicle.getSpeed('LT')
         vel_FT = traci.vehicle.getSpeed('FT')
         vel_LC = traci.vehicle.getSpeed('LC')
         vel_FC = traci.vehicle.getSpeed('FC')
         vel_AV = traci.vehicle.getSpeed('AV')
         
         K = ChooseGain(vel_AV)
         
         global safe1, lane_change_id, time_all, long_accl_AV_all, Xk_AV, e, y_ang, pos_all, lane_change_flag, vel_all, accel_auto_all, y_k, e40, safe_dist, pos_y_all, time_TTC_FT_AV_LT, time_TTC_LC_AV
         global DxxS, IxxS, IxuS, P, u, change_gain, accelFT, if_des_lane, initateLaneChange, initateLaneAbort, randNum, e20, agg_gain_time2, ttc_AV_FT_all, ttc_LT_AV_all, ttc_LC_AV_all, St
         
         if traci.simulation.getTime() > 2:
             ## ---------------------------------- This block checks for safety  ---------------------------------- ##
             if (Pos_AV_x <= Pos_LT_x-(h*vel_LT+distStand+carW)) and (Pos_AV_x >= Pos_FT_x+(h*vel_FT+distStand+carW)) and (Pos_AV_x <= Pos_LC_x-(h*vel_LC+distStand+carW)) and (Pos_AV_x >= Pos_FC_x+(h*vel_FC+distStand+carW)) and (if_des_lane == 0):
             # if (Pos_AV_x >= Pos_FT_x+h*vel_FT) and (Pos_AV_x <= Pos_LC_x-h*vel_LC) and (Pos_AV_x >= Pos_FC_x+h*vel_FC) and (if_des_lane == 0):
                 lane_change_start_time = traci.simulation.getTime()
                 count = 0
                 ifLEarned = 1
                 LC_init = 1 # to mark the initiation of a lane change
                 # print("Lane change initiated at:", lane_change_start_time, "randNum", randNum, "if_des_lane:", if_des_lane)
                 if initateLaneChange == 1:
                     initateLaneChange = 0 # to avoid re-initialization
                     initateLaneAbort = 1  # in case its not safe, need to re initialize lane abort
                     decideAbort = 1
                     # y_k0C = np.array([-Pos_AV_y+Pos_LT_y, -vel_AV+vel_LT, angle_AV-math.pi/2, 0])
                     y_k0C = np.array([-Pos_AV_y+Pos_LT_y, e20, angle_AV-math.pi/2, 0])
                     a = np.kron(y_k0C.T, y_k0C.T).tolist()
                     b = np.kron(y_k0C.T, 0).tolist()
                     y_k0C = np.append(y_k0C.tolist(),a)
                     y_k0C = np.append(y_k0C,b)
                 
                 LeaderID = "LT"
                 Leader_x = Pos_LT_x
                 Leader_y = Pos_LT_y
                 Leader_vel = vel_LT
                 
                 Xk_AV0 = np.array([Pos_AV_x, vel_AV]) # initial value
                 Xk_FT0 = np.array([[Pos_FT_x], [vel_FT]]) # initial value
                 Xk_LC0 = np.array([[Pos_LC_x], [vel_LC]]) # initial value
                 Xk_FC0 = np.array([[Pos_FC_x], [vel_FC]]) # initial value
                 Xk_LT0 = np.array([[Pos_LT_x], [vel_LT]]) # initial value
                 time = traci.simulation.getTime()
    
                 time_all = np.append(time_all,np.array([[time]]))
                 # Xk_AV = np.append(Xk_AV, Xk_AV0, axis=-1)
                 long_accl_AV_all = np.append(long_accl_AV_all, np.array([accel_Auto]),  axis=-1)
                 # accel_auto_all = np.append(accel_auto_all, np.array([accel_Auto]),  axis=-1)
                 # e = np.append(e, e0, axis=-1)
                 # y_ang = np.append(y_ang, np.array([[-e10+Pos_LT_y],[math.degrees(e30)+90]]), axis=-1)
                 
                 # pos_all = np.append(pos_all, np.array([[Pos_AV_x],[Pos_LT_x],[Pos_FT_x],[Pos_LC_x],[Pos_FC_x]]), axis=-1)
                 # pos_y_all = np.append(pos_y_all, np.array([[Pos_AV_y],[Pos_LT_y],[Pos_FT_y],[Pos_LC_y],[Pos_FC_y]]), axis=-1)
                 # vel_all = np.append(vel_all, np.array([[vel_AV],[vel_LT],[vel_FT],[vel_LC],[vel_FC]]), axis=-1)
                 
                 # safe_dist = np.append(safe_dist, np.array([[h*vel_LT],[h*vel_FT],[h*vel_LC],[h*vel_FC]]), axis=-1)
             elif (if_des_lane == 1):
                 LeaderID = "LT"
                 print("if_des_lane: ", if_des_lane, "LeaderID: ", LeaderID)
                 Leader_x = Pos_LT_x
                 Leader_y = Pos_LT_y
                 Leader_vel = vel_LT
                 Xk_AV0 = np.array([Pos_AV_x, vel_AV]) # initial value
                 Xk_FT0 = np.array([[Pos_FT_x], [vel_FT]]) # initial value
                 Xk_LC0 = np.array([[Pos_LC_x], [vel_LC]]) # initial value
                 Xk_FC0 = np.array([[Pos_FC_x], [vel_FC]]) # initial value
                 Xk_LT0 = np.array([[Pos_LT_x], [vel_LT]]) # initial value
                 if initateLaneFollow == 1:
                     initateLaneFollow = 0 # to avoid re-initialization
                     # initateLaneAbort = 1  # in case its not safe, need to re initialize lane abort
                     # y_k0C = np.array([-Pos_AV_y+Pos_LT_y, -vel_AV+vel_LT, angle_AV-math.pi/2, 0])
                     y_k0C = np.array([-Pos_AV_y+Pos_LT_y, e20, angle_AV-math.pi/2, 0])
                     a = np.kron(y_k0C.T, y_k0C.T).tolist()
                     b = np.kron(y_k0C.T, 0).tolist()
                     y_k0C = np.append(y_k0C.tolist(),a)
                     y_k0C = np.append(y_k0C,b)
             else: # Not safe and not in desired lane
                 count = 0
                 ifLEarned = 1
                 LC_init = 1 # to mark the abortion of a lane change
                 lane_abortion_time = traci.simulation.getTime()
                 # print("Lane change aborted at:", lane_abortion_time, "randNum", randNum, "if_des_lane:", if_des_lane)
                 if initateLaneAbort == 1:
                     initateLaneAbort = 0 # to avoid re-initialization
                     initateLaneChange = 1 # in case its  safe, need to re initialize lane change
                     # y_k0C = np.array([-Pos_AV_y+Pos_LC_y, 0, angle_AV-math.pi/2, 0])
                     y_k0C = np.array([-Pos_AV_y+Pos_LC_y, e20, angle_AV-math.pi/2, 0])
                     a = np.kron(y_k0C.T, y_k0C.T).tolist()
                     b = np.kron(y_k0C.T, 0).tolist()
                     y_k0C = np.append(y_k0C.tolist(),a)
                     y_k0C = np.append(y_k0C,b)
                 
                 # Necessary to define the initial conditions here as to start with, the AV might not be at a safe distance form other
                 # vehicles. In that case, if the initial conditions are not defined here, the the simulation with through error.
                 Xk_AV0 = np.array([Pos_AV_x, vel_AV]) # initial value
                 Xk_FT0 = np.array([[Pos_FT_x], [vel_FT]]) # initial value
                 Xk_LC0 = np.array([[Pos_LC_x], [vel_LC]]) # initial value
                 Xk_LT0 = np.array([[Pos_LT_x], [vel_LT]]) # initial value
                 lane_change_start_time = traci.simulation.getTime()
                 lane_change_end_time = 0
                 lane_change_flag = 0
                 
                 time = traci.simulation.getTime()
                 time_all = np.append(time_all,np.array([[time]]))
                 # Xk_AV = np.append(Xk_AV, Xk_AV0, axis=-1)
                 long_accl_AV_all = np.append(long_accl_AV_all, np.array([accel_Auto]),  axis=-1)
                 # accel_auto_all = np.append(accel_auto_all, np.array([accel_Auto]),  axis=-1)
                 # e = np.append(e, e0, axis=-1)
                 # y_ang = np.append(y_ang, np.array([[-e10+Pos_LT_y],[math.degrees(e30)+90]]), axis=-1)
                 
                 # pos_all = np.append(pos_all, np.array([[Pos_AV_x],[Pos_LT_x],[Pos_FT_x],[Pos_LC_x],[Pos_FC_x]]), axis=-1)
                 # pos_y_all = np.append(pos_y_all, np.array([[Pos_AV_y],[Pos_LT_y],[Pos_FT_y],[Pos_LC_y],[Pos_FC_y]]), axis=-1)
                 # vel_all = np.append(vel_all, np.array([[vel_AV],[vel_LT],[vel_FT],[vel_LC],[vel_FC]]), axis=-1)
    
                 LeaderID = "LC"
                 Leader_x = Pos_LC_x
                 Leader_y = Pos_LC_y
                 Leader_vel = max(vel_LC, vel_LC)
             ## ------------------------------------------------------------------------------------------------------------ ##
             
             time = traci.simulation.getTime()
             time_all = np.append(time_all,np.array([[time]]))
            
             safe_dist = np.append(safe_dist, np.array([[h*vel_LT+distStand+carW],[h*vel_FT+distStand+carW],[h*vel_LC+distStand+carW],[h*vel_FC+distStand+carW]]), axis=-1)
             # pos_all = np.append(pos_all, np.array([[Pos_AV_x],[Pos_LT_x],[Pos_FT_x],[Pos_LC_x],[Pos_FC_x]]), axis=-1)

             psi_des = math.pi/2
    
             t = np.linspace(time-dt,time,11)
             # print("vel_AV:", vel_AV)
             y_kC = odeint(model,y_k0C,t,atol=1e-10,rtol=1e-10,args=(K,1, (vel_AV)))
             
           
             y_kC0 = y_kC[0,:]
             y_kC1 = y_kC[-1,:]
           
             y_k0C = y_kC1
             
             e1 =  y_kC1[0]
             e2 =  y_kC1[1]
             e3 =  y_kC1[2]
             e4 =  y_kC1[3]
             e10 = e1
             e20 = e2
             e40 = e4
             e30 = e3
            
             e0 = np.array([[e1],[e2],[e3],[e4]])
             
             u_steer = -K@e0
             print("K:", K, "u:", u_steer)
             steer_angle_all = np.append(steer_angle_all, u_steer, axis=-1)
             #-----Data Collection-----#
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
                 
                 
             ## -------------------- longitudinal dynamics of autonomous vehicle ---------- ##
             
             if (LeaderID == "LC") and (abs(Pos_LC_y-Pos_AV_y)<=2.4):
                  Leader_vel = vel_LC
                  
             r = np.array([Leader_x-(h*vel_LT+distStand+carW), Leader_vel])
             Kl = np.array([[4.47213595499958,	110.382108140762]])
             if OutReg == 1:
                 # Kl = np.array([[0.0715262765604084,	0.384583524450610]])
                 # Ll = np.array([[0.0714426181991808,	0.390262863029821]])
                 Kl = np.array([[0.316235341317086,	0.855854112990615]])
                 Ll = np.array([[0.316235341317086,	0.855854112990615]])
                 ul = -Kl@Xk_AV0+Ll@r
                 long_accl_AV = ul
                 force_accel = np.append(force_accel, ul, axis=-1)
                 
             else:
                 ul = -Kl@Xk_AV0.reshape(2,1)+Kl@r.reshape(2,1)
                 # long_accl_AV = ul/m
                 long_accl_AV = ul/m
                 force_accel = np.append(force_accel, ul.reshape(1,)*(1/m), axis=-1)
             
             Xk_AV1 = odeint(model_longi,Xk_AV0,t,atol=1e-10,rtol=1e-10,args=(r, Kl, OutReg))
             Xk_AV0 = Xk_AV1[-1,:]

             ## ------------------------------------------------------------------------------------------------ ##

             
             ## ---------- This block decides when to make the longitudinal controller agressive and accelerate LT OR Use Output Regulation ---------- ##
             # if (UseOutReg == 1) and (traci.vehicle.getLaneID("AV") == "AB_2"):
             #     OutReg = 1
             
             if (traci.vehicle.getLaneID("AV") == "AB_2"):
                 if_des_lane = 1
                 
                 
             lane_change_const = 0.3
             if (abs(Pos_LT_y-Pos_AV_y)<=lane_change_const) and (Pos_AV_x > Pos_LT_x-h*vel_LT-7):
                      agg_gain_time2 = agg_gain_time2 + dt
                      print("Pos_LT_y-Pos_AV_y:", Pos_LT_y-Pos_AV_y)
                      change_gain = 1
                      if_des_lane = 1
                      weight = 0
                      weightCount = 0
             elif (abs(Pos_LC_y-Pos_AV_y)<=lane_change_const) and (Pos_AV_x> Pos_LC_x-h*vel_LC-7):
                      agg_gain_time2 = agg_gain_time2 + dt
                      print("Pos_LC_y-Pos_AV_y:", Pos_LC_y-Pos_AV_y)
                      change_gain = 1
                      weight = 0
                      weightCount = 0
             elif (decideAbort == 1) and (Pos_AV_x < Pos_FT_x+h*vel_FT+distStand+carW): #decided to abort as the FT is not at a safe distance form AV
                       change_gain = 1
                       weight = 0
                       weightCount = 0
             else:
                      print("Leader_y-Pos_AV_y:", Leader_y-Pos_AV_y)
                      change_gain = 0
                      # agg_gain_time2 = 0
                      AVLaneID = traci.vehicle.getLaneID("AV")
             print("LeaderID: ", LeaderID, (Pos_AV_x < Pos_FT_x+h*vel_FT+distStand+carW))
             print("h, AV_constant_vel:", h, AV_constant_vel)
             print("Aggresive gain time:", agg_gain_time2)
             print("Pos_LT_y-Pos_AV_y", Pos_LT_y-Pos_AV_y)
             print("---------------------------------")
             ## ------------------------------------------------------------------------------------------------ ##
             

                 
                 # Pos_AV_x = traci.vehicle.getPosition('AV')[0]
             count = count + 1
             ang = 90+math.degrees(e3)
                 
             traci.vehicle.moveToXY(vehID='AV',edgeID='AB', lane=2, x=Xk_AV0[0], y=-e1+Leader_y, angle=ang, keepRoute=2, matchThreshold=100000)
             # print("vel_AV", vel_AV, (AV_constant_vel==1), (abs(Leader_y-Pos_AV_y)>=0.3))
             if (aaaa==0):
                 const_vel_AV = max(vel_AV, max(vel_FT, vel_LT))
                 
             if (abs(Leader_y-Pos_AV_y)>=0.3) and (AV_constant_vel==1):
                 aaaa = 1
                 traci.vehicle.setSpeed('AV', const_vel_AV)
             else:
                 traci.vehicle.setSpeed('AV', Xk_AV0[1])
    
             if long_accl_AV>0:
                      traci.vehicle.setAccel('AV', long_accl_AV)
             else:
                      traci.vehicle.setDecel('AV', -long_accl_AV)
                 

             
             # collect data
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
             
             Pos_LT_y0 = Pos_LT_y
             Pos_LC_y0 = Pos_LC_y
             Pos_AV_y0 = Pos_AV_y
                 
             vel_LT = traci.vehicle.getSpeed('LT')
             vel_FT = traci.vehicle.getSpeed('FT')
             vel_LC = traci.vehicle.getSpeed('LC')
             vel_FC = traci.vehicle.getSpeed('FC')
             vel_AV = traci.vehicle.getSpeed('AV')
             
             print('vel AV = ', vel_AV)
                 
             accel_Auto = traci.vehicle.getAcceleration('AV')
                 
             angle_AV = traci.vehicle.getAngle('AV')
             angle_AV = math.radians(angle_AV)
             d1 = math.dist([Pos_AV_x, Pos_AV_y], [Pos_LT_x, Pos_LT_y])
             d2 = math.dist([Pos_AV_y], [Pos_LT_y])
             psi_des = math.pi/2

             pos_all = np.append(pos_all, np.array([[Pos_AV_x],[Pos_LT_x],[Pos_FT_x],[Pos_LC_x],[Pos_FC_x]]), axis=-1)
             St = np.append(St, np.array([[Pos_AV_x-Pos_LT_x-5],[Pos_FT_x-Pos_AV_x-5],[Pos_AV_x-Pos_LC_x-5],[Pos_FC_x-Pos_AV_x-5]]), axis=-1)
             pos_y_all = np.append(pos_y_all, np.array([[Pos_AV_y],[Pos_LT_y],[Pos_FT_y],[Pos_LC_y],[Pos_FC_y]]), axis=-1)
             vel_all = np.append(vel_all, np.array([[vel_AV],[vel_LT],[vel_FT],[vel_LC],[vel_FC]]), axis=-1)
             e = np.append(e, np.array([[e1],[e2],[e3],[e4]]), axis=-1)
             y_ang = np.append(y_ang, np.array([[Pos_AV_y],[angle_AV]]), axis=-1)
             # long_accl_AV_all = np.append(long_accl_AV_all,long_accl_AV,  axis=-1)
             accel_auto_all = np.append(accel_auto_all, np.array([accel_Auto]),  axis=-1)
                 # str_angle = np.append(str_angle, u, axis=-1)
                 
     
                 
             step += 1
     

     from scipy.io import savemat
     savemat("Robust_Lane_Changing_Gain_Scheduling_h_"+str(h)+".mat", {"Xk_AV":Xk_AV,"e":e, "y_ang":y_ang, "long_accl_AV_all":long_accl_AV_all, \
                                                   "time_all":time_all, "vel_all":vel_all, "accel_auto_all":accel_auto_all, \
                                                 "pos_all":pos_all, "lane_change_start_time":lane_change_start_time,\
                                                  "lane_change_end_time":lane_change_end_time, "lane_abortion_time":lane_abortion_time,\
                                                 "safe_dist":safe_dist,  "pos_y_all":pos_y_all})
     
     traci.close()
     sys.stdout.flush()
     


def get_options():
     optParser = optparse.OptionParser()
     optParser.add_option("--nogui", action="store_true",
                          default=False, help="run the commandline version of sumo")
     options, args = optParser.parse_args()
     return options


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
     
     traci.start([sumoBinary, "-c", "NGSIMTest.sumocfg"])
     run()
     

            
