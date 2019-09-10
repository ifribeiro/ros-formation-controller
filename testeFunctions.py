import math
import numpy as np 

odomx=1
odomy=1
odomz=2 
odomx2=0.5
odomy2=0.8
odomz2=1
odomphi = 0.0
a = 0.2


qdes = np.transpose([2, 1, 1.5, 1, 0, 0])

def controleFormacao(odomx=None, odomy=None, odomz=None, odomx2=None, odomy2=None, odomz2=None):
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
    q = np.array([xf, yf, zf, rhof, betaf, alphaf])

    qtil = (qdes - np.transpose(q))

    #Matriz de ganhos
    L1 = 0.2*np.identity(6)
    L2 = 0.3*np.identity(6)    

    #qrefp = L1*tanh(L2*qtil)

    #L2*qtil
    #retorna uma matriz com uma única columa .shape(6,)
    L2_x_qtil = L2.dot(qtil)
    
    #tanh(L2*qtil)
    tanh_l2_qtil = [math.tanh(x) for x in L2_x_qtil]

    qrefp = L1.dot(tanh_l2_qtil)


    jacob = np.array([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [1, 0, 0, math.cos(alphaf)*math.cos(betaf), -rhof*math.sin(alphaf)*math.cos(betaf), -rhof*math.cos(alphaf)*math.sin(betaf)],
                [0, 1, 0, math.sin(alphaf)*math.cos(betaf), rhof*math.cos(alphaf)*math.cos(betaf),  -rhof*math.sin(alphaf)*math.sin(betaf)],
                [0, 0, 1, math.sin(betaf), 0, rhof*math.cos(betaf)]])


    xrefp = jacob.dot(qrefp)


    K = np.array([
            [math.cos(odomz), math.sin(odomz), 0, 0, 0, 0],
            [-math.sin(odomz)/a, math.cos(odomz)/a, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, math.cos(odomphi), -math.sin(odomphi), 0],
            [0, 0, 0, math.sin(odomphi), math.cos(odomphi), 0],
            [0, 0, 0, 0, 0, 1]])

    return K

q = controleFormacao(odomx=1, odomy=1, odomz=2, odomx2=0.5, odomy2=0.8, odomz2=1)

print (q)