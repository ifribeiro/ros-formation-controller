# -*- coding: utf-8 -*-
import math
import numpy as np
import time


def controleFormacao(odom_d1=[], odom_d2=[], qdes=None, K=[], Ku=[], Kv=[]):
    Vd1A = 0
    ku_inv = []
    odomx = odom_d1[0]
    odomy = odom_d1[1]
    odomz = odom_d1[2]
    odompsi = odom_d1[3]

    print ()


    odomx2 = odom_d2[0]
    odomy2 = odom_d2[1]
    odomz2 = odom_d2[2]
    odompsi2 = odom_d2[3]


    xf = odomx
    yf = odomy
    zf = odomz

    #rhof
    rhof = math.sqrt((odomx2 - odomx)**2 + (odomy2 - odomy)**2 + (odomz2 - odomz)**2)

    #alphaf
    alphaf =  math.atan2((odomy2 - odomy), (odomx2-odomx))

    #betaf
    betaf = math.atan2((odomz2 - odomz), math.sqrt((odomx2 - odomx)**2 + (odomy2 - odomy)**2))

    #q
    q = np.array([xf, yf, zf, rhof, alphaf, betaf])
    #qtil
    qtil = qdes - np.transpose(q)


    #Matriz de ganhos
    L1 = 0.5*np.identity(6)
    L2 = 0.5*np.identity(6)   

    #qrefp = L1*tanh(L2*qtil)

    #L2*qtil
    #retorna uma matriz com uma única columa .shape(6,)
    L2_x_qtil = L2.dot(qtil)
    
    #tanh(L2*qtil)
    tanh_l2_qtil = [math.tanh(x) for x in L2_x_qtil]

    qrefp = L1.dot(tanh_l2_qtil)

    t = time.time()

    jacob = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [1, 0, 0, math.cos(alphaf)*math.cos(betaf), -rhof*math.sin(alphaf)*math.cos(betaf), -rhof*math.cos(alphaf)*math.sin(betaf)],
            [0, 1, 0, math.sin(alphaf)*math.cos(betaf), rhof*math.cos(alphaf)*math.cos(betaf),  -rhof*math.sin(alphaf)*math.sin(betaf)],
            [0, 0, 1, math.sin(betaf), 0, rhof*math.cos(betaf)]])


    xrefp = jacob.dot(qrefp)


    F1 = np.array([
        [math.cos(odompsi), -math.sin(odompsi), 0, 0],
        [math.sin(odompsi), math.cos(odompsi), 0, 0],
        [0,     0,      1,      0],
        [0,     0,      0,      1]
    ])

    F2 = np.array([
        [math.cos(odompsi2), -math.sin(odompsi2), 0, 0],
        [math.sin(odompsi2), math.cos(odompsi2), 0, 0],
        [0,     0,      1,      0],
        [0,     0,      0,      1]
    ])

    #Calcula as inversas das matrizes F1 e F2
    print (F1)
    F1 = np.linalg.inv(F1)
    F2 = np.linalg.inv(F2)

    print ("F1_INV", F1)
    print ("F2_INV", F2)

    Xd1p = np.append(xrefp[:3,], 0)
    Xd2p = np.append(xrefp[3:,], 0)
    
    print ("Xd2p",Xd2p)
    
    Vd1 = F1.dot(Xd1p)
    Vd2 = F2.dot(Xd2p)

    print ("Vd1 ", Vd1)

    #Deriva a velocidade
    Vd1p = (Vd1 - Vd1A)/(2)
    Vd2p = (Vd2 - Vd1A)/(2)

    print ("Vd2p ", Vd2p)
    Vd1A = Vd1
    Vd2A = Vd2
    print ("Vd1A ",odom_d1[4],odom_d1[5], odom_d1[6])
    print ("Odom", odom_d2[4],odom_d2[5], odom_d2[6])
    vel_bp1 = F1.dot(np.transpose([odom_d1[4],odom_d1[5], odom_d1[6], 0]))
    vel_bp2 = F2.dot(np.transpose([odom_d2[4],odom_d2[5], odom_d2[6], 0]))
    print ("Vel_bp1", vel_bp1)
    Ku_inv = np.linalg.inv(Ku)

    Ud1 = (Ku_inv.dot((Vd1p+K.dot(Vd1-vel_bp1) + Kv.dot(vel_bp1))))
    Ud2 = (Ku_inv.dot((Vd2p+K.dot(Vd2-vel_bp2) + Kv.dot(vel_bp2))))
    print (Ud1)
    print (Ud2)

    #Ud1 = Ku\(dVd1 + K*(Vd1 - Vb1) + Kv*Vb1);
    #Ud2 = Ku\(dVd2 + K*(Vd2 - Vb2) + Kv*Vb2);


    #vel = np.array([odom_d1[4], odom_d1[5], odom_d1[6], odom_d1[7], odom_d2[4], odom_d2[5], odom_d2[6], odom_d2[7]])

    return 0

odomx = 1
odomy = 2
odomz = 3

odomx2 = 2
odomy2 = 3
odomz2 = 4

qdes = np.transpose([0.0, 0.0, 1.5, 0.0, 0.0, 0.0])

odom_d1 = (1,2,3,0.5,1,1,1,1)
odom_d2 = (2,3,4,0.5,1,1,1,1)


Vd1A = 0
Vd2A = 0
K = np.diag([.8, .8, .8, .8])
Ku = np.diag([0.8417, 0.8354, 3.966, 9.8524])
Kv = np.diag([0.18227, 0.17095, 4.001, 4.7295])

v = controleFormacao(odom_d1, odom_d2, qdes=qdes, K=K, Ku=Ku, Kv=Kv)

 
t = time.time()
k = 0
for i in range(1, 10000):
    k = (k+i)*i
toc = time.time()-t



#print (toc)
#print ((Vd1- Vd1A)/(time.time()-t))

#vels = [odom_d1[0],odom_d1[1],odom_d1[2],odom_d1[3], odom_d2[0]]