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


def generate_routefile():
     # demand per second from different directions
     with open("ABTesting.rou.xml", "w") as routes: 
         # laneChangeModel= "SL2015"
         print("""<routes>
         <vType id="auto" accel="5" decel="4.5" length="5" minGap="1" maxSpeed="20" guiShape="passenger" color="1,0,0"/>
         <vType id="human" accel="5" decel="4.5"  length="5" minGap="1" maxSpeed="20" guiShape="passenger" color="0,0,1"/>
         <vType id="human1" accel="5" decel="4.5"  length="5" minGap="1" maxSpeed="20" guiShape="passenger" color="1,1,1"/>
         <vType id="human2" accel="5" decel="4.5"  length="5" minGap="1" maxSpeed="20" guiShape="passenger" color="0,1,0"/>
         <vType id="leader" accel="5" decel="4.5"  length="5" minGap="1" maxSpeed="20" guiShape="passenger" color="black"/>
         <route id="right" edges="AB BC"/>""", file=routes)
         
         
         # to campare with MPC
         print('<vehicle id="LT" type="human1" route="right" depart="0" departSpeed="20" departLane="2" departPos="257.5" />', file=routes)
         # print('<vehicle id="LT" type="human1" route="right" depart="0" departSpeed="20" departLane="2" departPos="260" />', file=routes)
         # print('<vehicle id="LT" type="human1" route="right" depart="0" departSpeed="20" departLane="2" departPos="300" />', file=routes)
         print('<vehicle id="FT" type="human1" route="right" depart="0" departSpeed="20" departLane="2" departPos="207.5" />', file=routes)
         print('<vehicle id="LC" type="human1" route="right" depart="0" departSpeed="20" departLane="1" departPos="262.5" />', file=routes)
         # print('<vehicle id="LC" type="human1" route="right" depart="0" departSpeed="20" departLane="1" departPos="300" />', file=routes)
         # print('<vehicle id="LC" type="human1" route="right" depart="0" departSpeed="20" departLane="1" departPos="320" />', file=routes)
         print('<vehicle id="AV" type="auto" route="right" depart="0" departSpeed="12" departLane="1" departPos="40" />', file=routes)
         print('<vehicle id="FC" type="human1" route="right" depart="0" departSpeed="12" departLane="1" departPos="13.5" />', file=routes)        
         
         print('<vehicle id="1" type="human2" route="right" depart="0" departSpeed="20" departLane="2" departPos="208" />', file=routes)
         print('<vehicle id="2" type="human2" route="right" depart="0" departSpeed="20" departLane="2" departPos="206" />', file=routes)
         print('<vehicle id="3" type="human2" route="right" depart="0" departSpeed="20" departLane="2" departPos="204" />', file=routes)
         print('<vehicle id="4" type="human2" route="right" depart="0" departSpeed="20" departLane="2" departPos="202" />', file=routes)
         print('<vehicle id="5" type="human2" route="right" depart="0" departSpeed="20" departLane="2" departPos="200" />', file=routes)
         print('<vehicle id="6" type="human2" route="right" depart="0" departSpeed="20" departLane="2" departPos="198" />', file=routes)
         print('<vehicle id="7" type="human2" route="right" depart="0" departSpeed="20" departLane="2" departPos="196" />', file=routes)
         print('<vehicle id="8" type="human2" route="right" depart="0" departSpeed="20" departLane="2" departPos="194" />', file=routes)
         print('<vehicle id="9" type="human2" route="right" depart="0" departSpeed="20" departLane="2" departPos="192" />', file=routes)
         print('<vehicle id="10" type="human2" route="right" depart="0" departSpeed="20" departLane="2" departPos="190" />', file=routes)
         
         
         print("</routes>", file=routes)

def learning_K(P, K, DxxS, IxxS, IxuS):
    

    epsi = 1e-4;
    DxxN = DxxS
    IxxN = IxxS
    IxuN = IxuS
    
    DxxN = DxxN
    it = 0; 
    POld = np.zeros(4)
    R = 1
    Q = np.diag([1, 2, 2000, 3000])
    K = K.reshape(1,4)
    # print('K=',K)
    while (LA.norm(P-POld) > epsi) and it<100:
        it = it + 1; 
        POld = P;   

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
        
        K = PK[16:20]
        K = K.reshape(1,4)

    print('learning iteration:',it, 'rank Theta=', LA.matrix_rank(Theta))
    
    return P,K

def TTC(h, Vl, Vf):
    #Vl = leader velocity
    #Vf = follower velocity
    if Vf <= Vl:
        ttc = np.nan
    else:
        ttc = h/(Vf-Vl)
        
    return ttc

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

def ChooseGain(vel_AV):
    
         ## The following gains are obtained for Q = diag([20 50 2000 3000]), R = 1
         K12L = np.array([[4.47213595659741,	1.15702287940076,	105.972901537432,	53.8686516719172]])
         K13L = np.array([[4.47213595540370,	1.18975252180826,	111.533682315610,	53.8521554154577]])
         K14L = np.array([[4.47213595478837,	1.22336884972174,	117.052908215672,	53.8323312716093]])
         K15L = np.array([[4.47213595576613,	1.25790330558084,	122.524449139833,	53.8096778187901]])
         K16L = np.array([[4.47213595481958,	1.29338829318294,	127.942550769076,	53.7846471429803]])
         K17L = np.array([[4.47213595498420,	1.32974138995633,	133.303507641889,	53.7574644286651]])
         K18L = np.array([[4.47213595479866,	1.36695662129205,	138.602970753796,	53.7284542494454]])
         K19L = np.array([[4.47213595488796,	1.40494844561908,	143.838260050863,	53.6977513494203]])
         
         # K20L = np.array([[4.47213595499958,	1.44367653964264,   149.006385361161,	53.6655683002131]])
         # K20p5L = np.array([[4.47213595499958,	1.46330233558858	,151.564447994438,	53.6489795604414]])
         # K21L = np.array([[4.47213595499958,	1.48309144352928	,154.104853021076,	53.6320799423933]])
         # K21p5L = np.array([[4.47213595499957,	1.50303672967695	,156.627362185258,	53.6148866975419]])
         # K22L = np.array([[4.47213595499959,	1.52313104309674,	159.131758544951	, 53.5974161827215]])
         # K22p5L = np.array([[4.47213595499963,	1.54336723595436	,161.617845525305,	53.5796839410106]])
         
         K20L = np.array([[4.47246979684194,	1.40434524828558,	149.405411456461,	53.7070574748389]])
         K20p5L = np.array([[4.47642432292582,	1.48078062317903,	151.176791145240,	53.6262471528589]])
         K21L = np.array([[4.47898728481299,	1.49044245445328,	154.117535127051,	53.6274224897734]])
         K21p5L = np.array([[4.42347209502289,	1.49837870195456,	156.352694605179,	53.6180547268714]])
         K22L = np.array([[4.48857749916987, 1.51678399885629,	159.150456234291,	53.6013130823015]])
         K22p5L = np.array([[4.47224702744857,	1.53286857123446,	161.463382979316,	53.5878694816464]])
         
         K23L = np.array([[4.47213595513767,	1.56373790935925,	164.085454042544,	53.5617058689653]])
         K23p5L = np.array([[4.47213595404799,	1.58423313680760,	166.534486190896,	53.5434810886800]])
         K24L = np.array([[4.47213595503623,	1.60485595560749,	168.964573060861,	53.5250617354031]])
         K24p5L = np.array([[4.47213595552925,	1.62559142845657,	171.375773811014,	53.5064360235338]])
         K25L = np.array([[4.47213595498725,	1.64642848567564,	173.768077422341,	53.4875924718181]])
         K25p5L = np.array([[4.47213595548553,	1.66736774227320,	176.141217846598,	53.4685764145106]])
         K26L = np.array([[4.47213595509902,	1.68840235133575,	178.495124685098,	53.4493979580293]])
         K28L = np.array([[4.47213595360906,	1.77333252101243,	187.718071589854,	53.3711441548730]])
         
         ## The following gains are obtained for Q = diag([20 50 2000 3000]), R = 0.1
         # K12L = np.array([[14.1421356237309,	3.71726660145978,	334.404697518580,	170.717684007283]])
         # K13L = np.array([[14.1421356237303,	3.81612784803828,	351.990314566577,	170.634199095552]])
         # K14L = np.array([[14.1421356237306,	3.91865894338135,	369.441862059433,	170.544721489767]])
         # K15L = np.array([[14.1421356237310,	4.02469813050507,	386.741206730005,	170.449870028337]])
         # K16L = np.array([[14.1421356237308,	4.13406614901286,	403.872609816525,	170.350167394524]])
         # K17L = np.array([[14.1421356237312,	4.24657191398471,	420.822400235092,	170.246067856791]])
         # K18L = np.array([[14.1421356237307,	4.36201670375154,	437.578720230426,	170.137975359647]])
         # K19L = np.array([[14.1421356237311,	4.48019736382299,	454.131322391086,	170.026255561300]])
         
         # ## The following gains are obtained for Q = diag([20 50 2000 3000]), R = 0.001
         # K20L = np.array([[4472.13595494784,	1460.13993498339	,148669.695509303,	53760.6956648007]])
         # K20p5L = np.array([[4472.13595500872,	1479.38536745469,	151227.142781050	,53741.2485388035]])
         # K21L = np.array([[4472.13595500359,	1498.81242522328,	153766.928027038	,53721.6177070157]])
         # K21p5L = np.array([[4472.13595498109,	1518.41268901889	,156288.813177449,	53701.8116885705]])
         # K22L = np.array([[4472.13595501115,	1538.17783906544	,158792.581480171,	53681.8389010358]])
         # K22p5L = np.array([[4472.13595499271,	1558.09966237983	,161278.036547506,	53661.7076529430]])
         # K23L = np.array([[4472.13595499637,	1578.17005963050,	163745.001467720,	53641.4261377106]])
         # K23p5L = np.array([[4472.13595499589,	1598.38105132454	,166193.317940738,	53621.0024266409]])
         # K24L = np.array([[4472.13595493558,	1618.72478355821,	168622.845463621,	53600.4444640214]])
         
         # K24p5L = np.array([[14.1421356237306,	5.16999059016288,	541.197368462743,	169.360838890622]])
         # K25L = np.array([[14.1421356237310,	5.23534364993990,	548.760891551108,	169.297022163457]])
         # K25p5L = np.array([[14.1421356237309,	5.30103442540293,	556.263982434941,	169.232782765498]])
         # K26L = np.array([[14.1421356237306,	5.36704040654003,	563.706397943704,	169.168148041380]])
         # K28L = np.array([[14.1421356237311,	5.63378111249205,	592.865969873138,	168.906176232154]])
         
         
         if (vel_AV<=12):
             print("vel_AV", vel_AV, ", in: <= 12")
             K = K12L
         
         if (vel_AV > 12) and (vel_AV<=13):
             print("vel_AV", vel_AV, ", in: 12 to 13")
             K = K12L + (K13L-K12L)*( (vel_AV-12)/((13-12)) )
             
         if (vel_AV > 13) and (vel_AV<=14):
             print("vel_AV", vel_AV, ", in: 13 to 14")
             K = K13L + (K14L-K13L)*( (vel_AV-13)/((14-13)) )
             
         if (vel_AV > 14) and (vel_AV<=15):
             print("vel_AV", vel_AV, ", in: 14 to 15")
             K = K14L + (K15L-K14L)*( (vel_AV-14)/((15-14)) )
             
         if (vel_AV > 15) and (vel_AV<=16):
             print("vel_AV", vel_AV, ", in: 15 to 16")
             K = K15L + (K16L-K15L)*( (vel_AV-15)/((16-15)) )
             
         if (vel_AV > 16) and (vel_AV<=17):
             print("vel_AV", vel_AV, ", in: 16 to 17")
             K = K16L + (K17L-K16L)*( (vel_AV-16)/((17-16)) )
             
         if (vel_AV > 17) and (vel_AV<=18):
             print("vel_AV", vel_AV, ", in: 17 to 18")
             K = K17L + (K18L-K17L)*( (vel_AV-17)/((18-17)) )
             
         if (vel_AV > 18) and (vel_AV<=19):
             print("vel_AV", vel_AV, ", in: 18 to 19")
             K = K18L + (K19L-K18L)*( (vel_AV-18)/((19-18)) )
             
         if (vel_AV > 19) and (vel_AV<=20):
             print("vel_AV", vel_AV, ", in: 19 to 20")
             K = K20L + (K20L-K19L)*( (vel_AV-19)/((20-19)) )
             
             
        # spacing = 2
             
         # if (vel_AV>20) and (vel_AV<=22):
         #           print("vel_AV", vel_AV, ", in: 20 to 22")
         #           K = K20L + (K22L-K20L)*( (vel_AV-20)/((22-20)) )
                  
         # if (vel_AV>22) and  (vel_AV<=24):
         #           print("vel_AV", vel_AV, ", in: 22 to 24")
         #           K = K22L + (K24L-K22L)*( (vel_AV-22)/((24-22)) )
                  
         # if (vel_AV>24) and (vel_AV<=26):
         #           print("vel_AV", vel_AV, ", in: 24 to 26")
         #           K = K24L + (K26L-K24L)*( (vel_AV-24)/((26-24)) )
      
         # # spacing = 1
             
         # if (vel_AV>20) and (vel_AV<=21):
         #           print("vel_AV", vel_AV, ", in: 20 to 21")
         #           K = K20L + (K21L-K20L)*( (vel_AV-20)/((21-20)) )
         
                  
         # if (vel_AV>21) and  (vel_AV<=22):
         #           print("vel_AV", vel_AV, ", in: 21 to 22")
         #           K = K21L + (K22L-K21L)*( (vel_AV-21)/((22-21)) )
         
                  
         # if (vel_AV>22) and  (vel_AV<=23):
         #           print("vel_AV", vel_AV, ", in: 22 to 23")
         #           K = K22L + (K23L-K22L)*( (vel_AV-22)/((23-22)) )
        
                  
         # if (vel_AV>23) and (vel_AV<=24):
         #           print("vel_AV", vel_AV, ", in: 23 to 23.5")
         #           K = K23L + (K24L-K23L)*( (vel_AV-23)/((24-23)) )
                  
         # if (vel_AV>24) and (vel_AV<=25):
         #           print("vel_AV", vel_AV, ", in: 24 to 25")
         #           K = K24L + (K25L-K24L)*( (vel_AV-24)/((25-24)) )
        
         # if (vel_AV>25) and (vel_AV<=26):
         #           print("vel_AV", vel_AV, ", in: 25 to 26")
         #           K = K25L + (K26L-K25L)*( (vel_AV-25)/((26-25)) )
                  
         
             
         ## spacing = 0.5
         if (vel_AV>20) and (vel_AV<=20.5):
                    print("vel_AV", vel_AV, ", in: 20 to 20.5")
                    K = K20L + (K20p5L-K20L)*( (vel_AV-20)/((20.5-20)) )
                  
         if (vel_AV>20.5) and  (vel_AV<=21):
                    print("vel_AV", vel_AV, ", in: 20.5 to 21")
                    K = K20p5L + (K21L-K20p5L)*( (vel_AV-20.5)/((21-20.5)) )
                  
         if (vel_AV>21) and  (vel_AV<=21.5):
                    print("vel_AV", vel_AV, ", in: 21 to 21.5")
                    K = K21L + (K21p5L-K21L)*( (vel_AV-21)/((21.5-21)) )
                  
         if (vel_AV>21.5) and (vel_AV<=22):
                    print("vel_AV", vel_AV, ", in: 21.5 to 22")
                    K = K21p5L + (K22L-K21p5L)*( (vel_AV-21.5)/((22-21.5)) )
                  
         if (vel_AV>22) and  (vel_AV<=22.5):
                    print("vel_AV", vel_AV, ", in: 22 to 22.5")
                    K = K22L + (K22p5L-K22L)*( (vel_AV-22)/((22.5-22)) )
                  
         if (vel_AV>22.5) and (vel_AV<=23):
                    print("vel_AV", vel_AV, ", in: 22.5 to 23")
                    K = K22p5L + (K23L-K22p5L)*( (vel_AV-22.5)/((23-22.5)) )
                  
         if (vel_AV>23) and (vel_AV<=23.5):
                    print("vel_AV", vel_AV, ", in: 23 to 23.5")
                    K = K23L + (K23p5L-K23L)*( (vel_AV-23)/((23.5-23)) )
                  
         if (vel_AV>23.5) and (vel_AV<=24):
                    print("vel_AV", vel_AV, ", in: 23.5 to 24")
                    K = K23p5L + (K24L-K23p5L)*( (vel_AV-23.5)/((24-23.5)) )
                  
         if (vel_AV>24) and (vel_AV<=24.5):
                    print("vel_AV", vel_AV, ", in: 24 to 24.5")
                    K = K24L + (K24p5L-K24L)*( (vel_AV-24)/((24.5-24)) )
                  
         if (vel_AV>24.5) and (vel_AV<=25):
                    print("vel_AV", vel_AV, ", in: 24.5 to 25")
                    K = K24p5L + (K25L-K24p5L)*( (vel_AV-24.5)/((25-24.5)) )
                  
         if (vel_AV>25) and (vel_AV<=25.5):
                    print("vel_AV", vel_AV, ", in: 25 to 25.5")
                    K = K25L + (K25p5L-K25L)*( (vel_AV-25)/((25.5-25)) )
                  
         if (vel_AV>25.5) and (vel_AV<=26):
                    print("vel_AV", vel_AV, ", in: 25.5 to 26")
                    K = K25p5L + (K26L-K25p5L)*( (vel_AV-25.5)/((26-25.5)) )
                  
         if (vel_AV>26):
             print("vel_AV", vel_AV, ", in: > 26")
             K = K28L
             
         # K = K12L
                  
         return K
    

# def run(h, AV_constant_vel, UseOutReg, NonCop):
def run(h=0.65, AV_constant_vel=0, UseOutReg = 0, NonCop = 1):
     """execute the TraCI control loop"""
     step = 0
     N1 = 8000
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
     while (step < N1):
         sim_start_time = traci.simulation.getTime()
         
         traci.simulationStep()    

         
         traci.vehicle.setSpeedMode('LT', 31)
         traci.vehicle.setSpeedMode('FT', 0)
         traci.vehicle.setSpeedMode('LC', 0)
         traci.vehicle.setSpeedMode('FC', 0)
         traci.vehicle.setSpeedMode('AV', 0)
         
         traci.vehicle.setSpeedMode('1', 0)
         traci.vehicle.setSpeedMode('2', 0)
         traci.vehicle.setSpeedMode('3', 0)
         traci.vehicle.setSpeedMode('4', 0)
         traci.vehicle.setSpeedMode('5', 0)

         traci.vehicle.setLaneChangeMode('LT', 0)
         traci.vehicle.setLaneChangeMode('FT', 1)
         traci.vehicle.setLaneChangeMode('LC', 1)
         traci.vehicle.setLaneChangeMode('FC', 1)
         traci.vehicle.setLaneChangeMode('AV', 1)
         
         traci.vehicle.setLaneChangeMode('1', 0)
         traci.vehicle.setLaneChangeMode('2', 0)
         traci.vehicle.setLaneChangeMode('3', 0)
         traci.vehicle.setLaneChangeMode('4', 0)
         traci.vehicle.setLaneChangeMode('5', 0)
         traci.vehicle.setLaneChangeMode('6', 0)
         traci.vehicle.setLaneChangeMode('7', 0)
         traci.vehicle.setLaneChangeMode('8', 0)
         traci.vehicle.setLaneChangeMode('9', 0)
         traci.vehicle.setLaneChangeMode('10', 0)

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
         
         traci.vehicle.setSpeed('1', vel_FT)
         traci.vehicle.setSpeed('2', vel_FT)
         traci.vehicle.setSpeed('3', vel_FT)
         traci.vehicle.setSpeed('4', vel_FT)
         traci.vehicle.setSpeed('5', vel_FT)
         traci.vehicle.setSpeed('6', vel_FT)
         traci.vehicle.setSpeed('7', vel_FT)
         traci.vehicle.setSpeed('8', vel_FT)
         traci.vehicle.setSpeed('9', vel_FT)
         traci.vehicle.setSpeed('10', vel_FT)

         accel_Hum = traci.vehicle.getAcceleration('LT')
         # accel_Hum2 = traci.vehicle.getAcceleration('FT')
         accel_Auto = traci.vehicle.getAcceleration('AV')

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
         # h = 0.5; # time headway (0.5, 0.6, 0,75, 0.85)
         # alpha = 12.5 # determines speed of FT (best)
         alpha = 15 # determines speed of FT
         weightHorizon = 300 # 500 = 5 seconds
         
         # print(h*vel_LC, h*vel_FC, h*vel_LT, h*vel_FT)
         dt = 0.01;
         
         global safe1, lane_change_id, time_all, long_accl_AV_all, Xk_AV, e, y_ang, pos_all, lane_change_flag, vel_all, accel_auto_all, y_k, e40, safe_dist, pos_y_all, time_TTC_FT_AV_LT, time_TTC_LC_AV
         global DxxS, IxxS, IxuS, P, u, change_gain, accelFT, if_des_lane, initateLaneChange, initateLaneAbort, randNum, e20, agg_gain_time2, ttc_AV_FT_all, ttc_LT_AV_all, ttc_LC_AV_all, St
         
         # AV_constant_vel = 0 # Set = 0 when want to use gain scheduling (varying Vx), set = 1 when do not want to use gain scheduling (constant Vx)
         
         # print("------------------------------------")
         # print("LT-FT: ", Pos_LT_x-Pos_FT_x)
         # print("------------------------------------")
         K = ChooseGain(vel_AV)
     
         if change_gain == 0:
             # Kl = np.array([[0.577350269189626,	39.8379140877438]])
             # Kl = np.array([[0.223606797749979,	24.6629375760461]])
             Kl = np.array([[4.47213595499958,	110.382108140762]])
             # Kl = np.array([[0.316235341317086,	0.855854112990615]])
             # Kl = np.array([[10.0000000000000, 165.227116418583]])
             print("normal gain")
         elif (change_gain == 1) and (OutReg == 0):
             # Kl = np.array([[10.0000000000000, 165.227116418583]])
             if weightCount <= weightHorizon:
                 weightCount = weightCount + 1
                 weight = weightCount/weightHorizon
             Kl1 = np.array([[4.47213595499958,	110.382108140762]])
             # Kl2 = np.array([[100.000000000000,	531.036721894071]])
             Kl2= np.array([[141.421356237309, 636.133703686168]])
             Kl = (1-weight)*Kl1 + weight*Kl2
             # Kl = Kl2
             Kl = np.array([[100.000000000000,	531.036721894071]])
             # Kl = Kl1
             # Kl = np.array([[0.316235341317086,	0.855854112990615]])
             print("aggressive gain", "OutReg: ", OutReg)
             
         # Kl = np.array([[100.000000000000,	531.036721894071]])
             
        
         
         # Kl = np.array([[4.47213595499958,	110.382108140762]])
         # Kl = np.array([[10.0000000000000, 165.227116418583]])
         Al = np.array([[ 0, 1 ],
                            [ 0, 0 ]])
             
         Bl = np.array([[0],[1/m]])
             
         Al = (Al-Bl@Kl)
         Bl = Bl@Kl
             
         Ald = expm(Al*dt)
         Id = np.identity(2)
         Bld = inv(Al)@(Ald-Id)@Bl
                  
         # When lane change is inititate for the very first time, we need to set the initial conditions w.r.t the LT.
         # When lane abortion is inititate, we need the set initial conditon w.r.t the LC
         # Remember: Whenever the AV decides to do lane change or abortion only them the initial conditions needs to be 
         # reset. Once the initial conditions are reset, we drive the AV with these initial conditions until the next
         # decision on lane change or abortion is taken.
         
         Kl_hum= np.array([[141.421356237309, 636.133703686168]])
         # print("vel_LT: ", vel_LT)
         # print("vel_FT: ", vel_FT)
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
             
                 
             if (count == 500) and (ifLEarned==0):
                     learning_start_time = traci.simulation.getTime()
                     P,K = learning_K(P, K, DxxS, IxxS, IxuS)
                     learning_end_time = traci.simulation.getTime()
                     print("K=",K)
                     print("learning time:", learning_end_time-learning_start_time)
                     count = 0
                     ifLEarned=1
                 
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
                 
             # traci.vehicle.setSpeed('LT', vel_LT)
             # traci.vehicle.setSpeed('LC', vel_LC)
             Al = np.array([[ 0, 1 ],
                                [ 0, 0 ]])
                 
             Bl = np.array([[0],[1/m]])
                 
             # print("-------------------------")
             # print("Pos_LT_y-Pos_AV_y:", Pos_LT_y-Pos_AV_y)
             # print("-------------------------")
             
             
             ## ---------- This block decides when to make the FT non-cooperative and computes TTC for (LT,AV), (AV,FT)---------- ##
             # if (NonCop == 1) and (abs(Pos_LT_y-Pos_AV_y)<=3.2) and (if_des_lane != 1) and (abs(Pos_LT_y-Pos_AV_y)>lane_change_const):
             if (NonCop == 1) and (abs(Pos_LT_y-Pos_AV_y)<=3.2) and (if_des_lane != 1) and (FT_aggressive_count < 250):
                   # randNum = 0.4
                   # print("in")
                   accelFT = 1  
                   rFT = np.array([[ Pos_LT_x-alpha],[vel_LT]])
                   ulH2 = Kl_hum@(-Xk_FT0+rFT)  
                   Xk_FT = Xk_FT0 + Al@Xk_FT0*dt + Bl@ulH2*dt
                   # Xk_AVH2 = np.append(Xk_AVH2, Xk_FT, axis=-1)
                   Xk_FT0 = Xk_FT
                   traci.vehicle.moveToXY(vehID='FT',edgeID='AB', lane=2, x=Xk_FT[0], y=Pos_LT_y, angle=90, keepRoute=2, matchThreshold=100000)
                   traci.vehicle.setSpeed('FT', Xk_FT[1])
                   FT_aggressive_count = FT_aggressive_count + 1
                   #compute TTC
                   time_TTC_FT_AV_LT = np.append(time_TTC_FT_AV_LT, np.array([traci.simulation.getTime()]),  axis=-1)
             elif accelFT == 1:
                   rFT = np.array([[ Pos_LT_x-20 ],[vel_LT-4]])
                   ulH2 = Kl_hum@(-Xk_FT0+rFT)  
                   Xk_FT = Xk_FT0 + Al@Xk_FT0*dt + Bl@ulH2*dt
                   # Xk_AVH2 = np.append(Xk_AVH2, Xk_FT, axis=-1)
                   Xk_FT0 = Xk_FT
                   traci.vehicle.moveToXY(vehID='FT',edgeID='AB', lane=2, x=Xk_FT[0], y=Pos_LT_y, angle=90, keepRoute=2, matchThreshold=100000)
                   traci.vehicle.setSpeed('FT', Xk_FT[1])
             ## ------------------------------------------------------------------------------------------------ ##
             ttc_AV_FT = TTC(h, vel_AV, vel_FT)
             ttc_LT_AV = TTC(h, vel_LT, vel_AV)
             ttc_AV_FT_all = np.append(ttc_AV_FT_all, np.array([ttc_AV_FT]),  axis=-1)
             ttc_LT_AV_all = np.append(ttc_LT_AV_all, np.array([ttc_LT_AV]),  axis=-1)
             ## ---------- This block computes TTA, total abortion distance traversed, and TTC for (LC,AV) ---------- ##
             if (abs(Pos_LC_y-Pos_AV_y)>0.1):
                 TTA = TTA + dt
                 pos_AV_x0 = traci.vehicle.getPosition('AV')[0]
                 ttc_LC_AV = TTC(h, vel_LC, vel_AV)
                 ttc_LC_AV_all = np.append(ttc_LC_AV_all, np.array([ttc_LC_AV]),  axis=-1)
                 time_TTC_LC_AV = np.append(time_TTC_LC_AV, np.array([traci.simulation.getTime()]),  axis=-1)
             if aaa == 0:
                 aaa = 1
                 pos_AV_x00 = traci.vehicle.getPosition('AV')[0]
             ## ------------------------------------------------------------------------------------------------ ##
             
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
     print("TTC", TTC)
     print("TTA", TTA)
     print("pos_AV_x0-pos_AV_x00", pos_AV_x0-pos_AV_x00)
     total_abort_dist = pos_AV_x0-pos_AV_x00
     
     savemat("Robust_Lane_Changing_Gain_Scheduling_h_"+str(h)+".mat", {"Xk_AV":Xk_AV,"e":e, "y_ang":y_ang, "long_accl_AV_all":long_accl_AV_all, \
                                                   "time_all":time_all, "vel_all":vel_all, "accel_auto_all":accel_auto_all, \
                                                 "pos_all":pos_all, "lane_change_start_time":lane_change_start_time,\
                                                  "lane_change_end_time":lane_change_end_time, "lane_abortion_time":lane_abortion_time,\
                                                 "safe_dist":safe_dist,  "pos_y_all":pos_y_all})
     
     # savemat("LaneChangeRiskIndexCop.mat",{"pos_all":pos_all, "pos_y_all":pos_y_all, "vel_all":vel_all, "St":St})

     
     # savemat("NonCop_"+str(NonCop)+"_UseOutReg_"+str(UseOutReg)+"_AV_constant_vel_"+str(AV_constant_vel)+"_h_"+str(h)+"_alpha_"+"More Fast"+".mat",{"ttc_AV_FT_all":ttc_AV_FT_all, "ttc_LT_AV_all":ttc_LT_AV_all, "Aggresive gain time":agg_gain_time2, "safe_dist":safe_dist, \
     #                                                                                                     "pos_all":pos_all, "force_accel":force_accel, "vel_all":vel_all, "e":e})


     #savemat("NonCop_"+str(NonCop)+"_UseOutReg_"+str(UseOutReg)+"_AV_constant_vel_"+str(AV_constant_vel)+"_h_"+str(h)+"_alpha_"+"More Fast"+".mat",{"safe_dist":safe_dist,"pos_y_all":pos_y_all, "accel_auto_all":accel_auto_all, "time_entry_des_lane":time_entry_des_lane,\
     #                                                                                                          "pos_all":pos_all, "force_accel":force_accel, "vel_all":vel_all, "e":e, "steer_angle_all":steer_angle_all, "Pos_LT_FT_LC_FC_AV":positions_vehicles})
     
     # savemat("NonCop_"+str(NonCop)+"_UseOutReg_"+str(UseOutReg)+"_AV_constant_vel_"+str(AV_constant_vel)+"_h_"+str(0.65)+"_alpha_"+"More Fast"+".mat",{"safe_dist":safe_dist,"pos_y_all":pos_y_all, "accel_auto_all":accel_auto_all, \
     #                                                                                                           "pos_all":pos_all, "force_accel":force_accel, "vel_all":vel_all, "e":e, "steer_angle_all":steer_angle_all, "Pos_LT_FT_LC_FC_AV":positions_vehicles})
     
     # AV_constant_vel = 1
     # savemat("NonCop_"+str(NonCop)+"_UseOutReg_"+str(UseOutReg)+"_AV_constant_vel_"+str(AV_constant_vel)+"_h_"+str(h)+"_alpha_"+"More Fast1_K15"+".mat",{"safe_dist":safe_dist,"pos_y_all":pos_y_all, "accel_auto_all":accel_auto_all, \
     #                                                                                                              "pos_all":pos_all, "force_accel":force_accel, "vel_all":vel_all, "e":e, "steer_angle_all":steer_angle_all})
     
     
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
     
     traci.start([sumoBinary, "-c", "ABCTesting.sumocfg"])
     run()
     

            
