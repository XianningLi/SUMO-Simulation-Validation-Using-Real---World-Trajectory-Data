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
directory = 'D:/Sayan Github/gain-scheduling-simulations/TestingNGSIM/trajectoryDataSet5'

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
FCPos = FC[['Local_Y', 'Local_X']]  # Note: Local_Y in data is Local_X here, and vice-versa
FCVel = FC['v_Vel']  
FCAccl = FC['v_Acc']  

LC = trajectories['veh_f.csv']
LCPos = LC[['Local_Y', 'Local_X']]  # Note: Local_Y in data is Local_X here, and vice-versa
LCVel = LC['v_Vel']  
LCAccl = LC['v_Acc'] 

LT = trajectories['veh_ft.csv']
LTPos = LT[['Local_Y', 'Local_X']]  # Note: Local_Y in data is Local_X here, and vice-versa
LTVel = LT['v_Vel'] 
LTAccl = LT['v_Acc']

FT = trajectories['veh_rt.csv']
FTPos = FT[['Local_Y', 'Local_X']]  # Note: Local_Y in data is Local_X here, and vice-versa
FTVel = FT['v_Vel']  
FTAccl = FT['v_Acc']

AV = trajectories['veh_s.csv']
AVPos = AV[['Local_Y', 'Local_X']]  # Note: Local_Y in data is Local_X here, and vice-versa
AVVel = AV['v_Vel']  
AVAccl = AV['v_Acc']
AVHead = AV['Time_Hdwy']


# Use this data to set vehicle positions and speeds in your SUMO simulation


def generate_routefile():
     # demand per second from different directions
     with open("NGSIMTest.rou.xml", "w") as routes: 
         # laneChangeModel= "SL2015"
         print("""<routes>
         <vType id="auto" accel="3.8" decel="3.8" length="4.4" width="1.5" minGap="1" maxSpeed="12" guiShape="passenger" color="1,0,0"/>
         <vType id="human" accel="3.8" decel="3.8"  length="4.4" width="1.5" minGap="1" maxSpeed="12" guiShape="passenger" color="0,0,1"/>
         <route id="right" edges="AB"/>""", file=routes)
           
         print('<vehicle id="AV" type="auto" route="right" depart="0" departSpeed="4.572" departLane="1" departPos="51.5285736" />', file=routes) # AV (s)
         print('<vehicle id="FC" type="human" route="right" depart="0" departSpeed="3.599688" departLane="1" departPos="35.4049584" />', file=routes) # FC (r) (1)
         print('<vehicle id="FT" type="human" route="right" depart="0" departSpeed="3.599688" departLane="0" departPos="21.8498928" />', file=routes) # FT (rt) (2)
         print('<vehicle id="LC" type="human" route="right" depart="0" departSpeed="3.048" departLane="0" departPos="63.5504952" />', file=routes) # LC (f) (3)
         print('<vehicle id="LT" type="human" route="right" depart="0" departSpeed="4.559808" departLane="0" departPos="58.1765664" />', file=routes) # LT (ft) (4)
    
         
         
         print("</routes>", file=routes)
         
         
# def run(h, AV_constant_vel, UseOutReg, NonCop):
def run():
     """execute the TraCI control loop"""
     
     
     step = 0
     N1 = 5980
     traci.simulationStep()        
     safe_dist_43 = np.array([[0],[0],[0],[0]])
     distStand = 2
     carW = 0.5
     while (step < N1):        
         
         # subject vehicle
         traci.vehicle.moveToXY('AV', edgeID='', lane=0, x=AVPos.loc[step, 'Local_Y']-2.2, y=AVPos.loc[step, 'Local_X']) # NOTE: X, Y positions are flipped to match orientation of the network
         traci.vehicle.setSpeed('AV', AVVel[step])
         # if  AVAccl[step]>0:
         #          traci.vehicle.setAccel('0', AVAccl[step])
         # else:
         #          traci.vehicle.setDecel('0', -AVAccl[step])
                  
         vel_AV = traci.vehicle.getSpeed('AV')
         accel_Auto = traci.vehicle.getAcceleration('AV')

          
         # followers
         traci.vehicle.moveToXY('FC', edgeID='', lane=1, x=FCPos.loc[step, 'Local_Y']-2.2, y=FCPos.loc[step, 'Local_X']) # NOTE: X, Y positions are flipped to match orientation of the network
         traci.vehicle.setSpeed('FC', FCVel[step])
         
                     
         traci.vehicle.moveToXY('FT', edgeID='', lane=0, x=FTPos.loc[step, 'Local_Y']-2.2, y=FTPos.loc[step, 'Local_X']) # NOTE: X, Y positions are flipped to match orientation of the network
         traci.vehicle.setSpeed('FT', FTVel[step])      
     
         
         # leaders
         traci.vehicle.moveToXY('LC', edgeID='', lane=1, x=LCPos.loc[step, 'Local_Y']-2.2, y=LCPos.loc[step, 'Local_X']) # NOTE: X, Y positions are flipped to match orientation of the network
         traci.vehicle.setSpeed('LC', LCVel[step])
         
         
         traci.vehicle.moveToXY('LT', edgeID='', lane=0, x=LTPos.loc[step, 'Local_Y']-2.2, y=LTPos.loc[step, 'Local_X']) # NOTE: X, Y positions are flipped to match orientation of the network
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
         
         #-------------------------------------------------#
         # print('Pos LT', Pos_LT_x, Pos_LT_y)
         # print('Pos FT', Pos_FT_x, Pos_FT_y)
         # print('Pos LC', Pos_LC_x, Pos_LC_y)
         # print('Pos FC', Pos_FC_x, Pos_FC_y)
         # print('Pos AV', Pos_AV_x, Pos_AV_y)
         print('Vel AV',  traci.vehicle.getSpeed('AV'))
         #-------------------------------------------------#
         safe_dist_43 = np.append(safe_dist_43, np.array([[AVHead[step]*LTVel[step]+distStand+carW],[AVHead[step]*FTVel[step]+distStand+carW],[AVHead[step]*LCVel[step]+distStand+carW],[AVHead[step]*FCVel[step]+distStand+carW]]), axis=-1)
         traci.simulationStep()    

         step += 1
     

     # from scipy.io import savemat
     from scipy.io import savemat
     savemat("NGSIM_SafeDist.mat", {"safe_dist_43":safe_dist_43})
     
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
     

            
