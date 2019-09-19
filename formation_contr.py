#! /usr/bin/env python3
# -*- coding: utf-8 -*-
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import rospy
import math
import tf


class Controller:
    def __init__(self):
        self.rospy=rospy
        self.rospy.init_node('listener',anonymous=True)
        self.rospy.loginfo("Iniciando o controle de formação") #" starting controle de formacao de drones"
        # self.initParameters()        
        self.initvariables() # inicializacao de variaveis
        self.initSubscribers() # topicos a serem subscritos por ex odometria
        self.initPublishers(self.bebop1_name, self.bebop2_name)
        self.change=False #label para indicar mudanca de variaveis        
        self.run()  # loop de controle

    def initSubscribers(self):
        """
        Inicia os Subscribers dos tópicos do ROS
        """

        self.subOdom = self.rospy.Subscriber('/%s/odom'%(self.bebop1_name), Odometry, self.get_drone_odom)
        self.subOdom2 = self.rospy.Subscriber('/%s/odom'%(self.bebop2_name), Odometry, self.get_drone2_odom)

        return

    def initPublishers(self, bebop1_name, bebop2_name):
        """    
        Inicia os Publishers onde serão publicas mensagens 

        Parâmetros: 
        bebop1_name, bebop2_name = nomes dos drones da formação definidos
        no namespace dos arquivos .launch do ROS

        """

        self.pubVel = rospy.Publisher('/%s/cmd_vel'%(bebop1_name), Twist, queue_size=10)
        self.pubVel2 = rospy.Publisher('/%s/cmd_vel'%(bebop2_name), Twist, queue_size=10)

        self.pubLand = rospy.Publisher('/%s/land'%(bebop1_name), Empty, queue_size=10)
        self.pubLand2 = rospy.Publisher('/%s/land'%(bebop2_name), Empty, queue_size=10)

        return

    def initvariables(self):
        """
        Inicia as várias que serão utilizadas no código
        """

        self.bebop1_name = 'bebop1'
        self.bebop2_name = 'bebop2'
        self.rate = self.rospy.Rate(10)
        self.Xd = np.transpose([[2, 1, 1.5, 0]])   
        #self.Xd = np.transpose([[8, 5.54, 0, 0]])

              
        self.ti = 0.1
        self.tfinal = 20
        self.t = np.arange(0, self.tfinal, self.ti)

        #Formação desejada
        self.qdes = np.transpose([2, 1, 1.5, 1, 0, 0])
        #self.qdes = np.transpose([[8, 5.54, 0, 1, 0, 0]])
        
        self.qtil = np.zeros((6, len(self.t)))

        self.Xb1 = np.zeros((4,len(self.t)))
        self.Xb2 = np.zeros((4,len(self.t)))

        #ks
        self.k1 = 4.3321
        self.k2 = 0.2333
        self.k3 = 1.1161
        self.k4 = 0.3786
        self.k5 = 4.2492
        self.k6 = 4.3235
        self.k7 = 10.4174
        self.k8 = 5.9794    

        #kps
        self.kpx = 1
        self.kpy = 0.5
        self.kpz = 0.7
        self.kpphi = 0.7
        self.kdx = 0.5
        self.kdy = 1
        self.kdz = 0.8
        self.kdphi = 0.7
        #kp
        self.kp = np.diag([self.kpx, self.kpy, self.kpz, self.kpphi])
        self.kd = np.diag([self.kdx, self.kdy, self.kdz, self.kdphi])


        self.ku = np.diag([self.k1, self.k3, self.k5, self.k7])
        self.kv = np.diag([self.k2, self.k4, self.k6, self.k8])
        self.K = np.diag([0.8, 0.8, 0.8, 0.8])

        #Condições iniciais
        #Drone 1
        self.x = 0*np.ones((len(self.t), 1))
        self.y = 0*np.ones((len(self.t),1))
        self.z = np.ones((len(self.t),1))
        self.phi = np.zeros((len(self.t),1))
        self.U = {}
        self.ganho = 0.4

        #(x, y, z, psi, vel_x, vel_y, velz)
        self.odom_drone = (0, 0, 1, 0, 0, 0 )
        
        
        self.old_odom_drone = (0,0,1,0)
        
        #(x, y, z, psi, vel_x, vel_y, velz)
        self.odom_drone2 = (1, 1, 1, 0)

        self.a=0.2
        self.v = {}
        self.odomx = 0*np.ones(len(self.t)-1)
        self.odomy = 0*np.zeros(len(self.t)-1)
        self.odomz = 1*np.zeros(len(self.t)-1)
        self.erro = np.zeros((3,len(self.t)-1))

        self.errosFormacao = np.zeros((3,len(self.t)-1))

        #Condições iniciais
        #Drone2

        #initial position Drone2
        self.xi = np.transpose([[-2, 1, 0.75, 0]])  

        self.x2 = -2*np.ones((len(self.t), 1))
        self.y2 = 1*np.ones((len(self.t), 1))
        self.z2 = 0.75*np.ones((len(self.t),1))
        self.phi2 = np.zeros((len(self.t),1))

    def get_drone_odom(self, msg):

        """
        Atualiza a odometria do Drone 1
        Parametros: 
        msg = Mensagem recebida do tópico
        """
        
        self.old_odom_drone = self.odom_drone

        #Posição
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        #Orientação em Quartenos
        xQ = msg.pose.pose.orientation.x
        yQ = msg.pose.pose.orientation.y
        zQ = msg.pose.pose.orientation.z
        wQ = msg.pose.pose.orientation.w
        quaterno = (xQ, yQ, zQ, wQ)
        euler = tf.transformations.euler_from_quaternion(quaterno)
        #Orientação
        psi = euler[2]

        vel_x = msg.twist.twist.linear
        vel_y = msg.twist.twist.linear
        vel_z = msg.twist.twist.linear

        #atualiza o valor da odometria
        self.odom_drone = (x, y, z, psi, vel_x, vel_y, vel_z)

        #verifica se a odometria mudou
        if(self.old_odom_drone != self.odom_drone):
            self.change = True
        return

    def get_drone2_odom(self,msg):
        """
        Atualiza a odometria do Drone 2
        Parametros: 
        msg = Mensagem recebida do tópico
        """

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        #Orientação em Quartenos
        xQ = msg.pose.pose.orientation.x
        yQ = msg.pose.pose.orientation.y
        zQ = msg.pose.pose.orientation.z
        wQ = msg.pose.pose.orientation.w
        quaterno = (xQ, yQ, zQ, wQ)
        euler = tf.transformations.euler_from_quaternion(quaterno)
        #Orientação
        psi = euler[2]

        vel_x = msg.twist.twist.linear
        vel_y = msg.twist.twist.linear
        vel_z = msg.twist.twist.linear

        #atualiza o valor da odometria
        self.odom_drone = (x, y, z, psi, vel_x, vel_y, vel_z)


    def takeOff(self, drone_name):

        """
        Inicia o vôo do drone

        Parametros: 
        drone_name = Nome do drone

        """

        pub = self.rospy.Publisher("/%s/takeoff"%(drone_name), Empty, queue_size=10)    
        rate_10 = self.rospy.Rate(10) # 10hz
        self.rospy.loginfo("Taking Off %s ..."%(drone_name))
        for i in range(1,25):
            pub.publish(Empty())
            rate_10.sleep()
        self.rospy.loginfo("Feito.")
        return True
        

    def land(self, drone_name):
        """
        Aterrissa o drone

        Parametros:
        drone_name = Nome do drone
        """

        pub = self.rospy.Publisher("/%s/land"%(drone_name), Empty, queue_size=10)    
        rate_10 = self.rospy.Rate(10) # 10hz
        self.rospy.loginfo("Landing %s ..."%(drone_name))
        for i in range(1,25):
            pub.publish(Empty())
            rate_10.sleep()
        self.rospy.loginfo("Feito.")
        return True
        

    def controlePosicionamento(self,X=None, Xd=None,Xtil=None, j=None, phi = None):
        """
        Realiza o controle de posicionamento do drone
        Parametros:
        Xd = Coordenadas da posição desejada
        Xtil = Erros entre a posição desejada e a atual
        j = incremento do tempo
        phi = Orientação do drone

        """

        f1 = [  
            [(self.k1)*(math.cos(phi[j])),(-self.k3)*(math.sin(phi[j])), 0, 0],
            [(self.k1)*math.sin(phi[j]), self.k3*math.cos(phi[j]), 0, 0],
            [0, 0, self.k5, 0],
            [0, 0, 0, self.k7]]

        f1_inv = np.linalg.inv(f1)        
        xd_x_to = Xd*self.ti        
        kd_Xtil = self.kd.dot(Xtil)
        tanh_kd_Xtil = np.transpose([np.array([math.tanh(xi) for xi in kd_Xtil])])                
        part3 = self.kp.dot(tanh_kd_Xtil)        
        xd_kp = xd_x_to + part3
        
        self.U[j] = ((f1_inv).dot(xd_kp))*self.ganho

        Ku_inv = np.linalg.inv(self.ku)

        
        Uponto = (self.U[j]-self.U[j-1])/self.ti

        Xponto = X - self.Xb1[:,j]
        

        print ("-----\n")
        print (Xponto,"\n")
        #print (X.shape,"\n")
        #print (self.X[:,j].shape)
        print ("-----\n")
        #print (Uponto[j][0][0], Uponto[j][0][0], Uponto[j][0][0]  )





        return self.U

    def controleFormacao(self, j=None, odomx=None, odomy=None, odomz=None, odomphi=None, odomx2=None, odomy2=None, odomz2=None, odomphi2=None):
        """
        Realiza o controle de formação

        Parâmetros:
        odomx,odomy,odomz,odomphi = Odometria do drone líder da formação
        odomx2,odomy2,odomz2 = Odometria do segundo drone da formação

        Retorna: a velocidade que deve ser aplicada ao segundo drone

        """

        self.Xb1[0,j] = odomx
        self.Xb1[1,j] = odomy
        self.Xb1[2,j] = odomz
        self.Xb1[3,j] = odomphi

        self.Xb2[0,j] = odomx2
        self.Xb2[1,j] = odomy2
        self.Xb2[2,j] = odomz2
        self.Xb2[3,j] = odomphi2


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

        x1 = xf
        y1 = yf
        z1 = zf 

        x2 = xf + rhof*math.cos(alphaf)*math.cos(betaf)
        y2 = yf + rhof*math.sin(alphaf)*math.cos(betaf)
        z2 = zf + rhof*math.sin(betaf)

        x = [x1, y1, z1, x2, y2, z2]


        print (x)

        #qtil
        qtil = (self.qdes - np.transpose(q))







        self.errosFormacao[:,j] = qtil[3:]

        #Matriz de ganhos
        L1 = 0.3*np.identity(6)
        L2 = 0.5*np.identity(6)   

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
            [-math.sin(odomz)/self.a, math.cos(odomz)/self.a, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, math.cos(odomphi), -math.sin(odomphi), 0],
            [0, 0, 0, math.sin(odomphi), math.cos(odomphi), 0],
            [0, 0, 0, 0, 0, 1]])

        
        v = K.dot(xrefp)
        
        x2 = odomx2+self.ti*v[0]
        y2 = odomy2+self.ti*v[1]
        z2 = odomz2+self.ti*v[2]


        return v


    def goToInitialPositionDrone2(self, bebop_name, xd=None, yd=None, zd=None, phi=0):
        """
        Faz o drone ir para a posição inicial

        Parametros:
        bebop_name = Nome do drone
        xd,yd,zd,phi = posição e orientação do drone

        """
        Xd = np.transpose([xd, yd, zd, phi])
        self.rospy.loginfo("Levando o %s para a posição inicial..."%(bebop_name))
        vel_msg = Twist()
        self.U[0] = [[self.odom_drone2[0] , self.odom_drone2[1], self.odom_drone2[2], self.odom_drone2[3]]]
        self.Xb1[0,0] = self.odom_drone[0]
        self.Xb1[1,0] = self.odom_drone[1]
        self.Xb1[2,0] = self.odom_drone[2]
        self.Xb1[3,0] = self.odom_drone[3]




        for i in range(1,len(self.t)-1):
            #X = np.transpose([[self.odom_drone2[0] , self.odom_drone2[1], self.odom_drone2[2], self.odom_drone2[3]]])

            #X = np.transpose([[self.x[i] , self.y[i], self.z[i], self.phi[i]]])
            self.Xb1[0,i] = self.odom_drone[0]
            self.Xb1[1,i] = self.odom_drone[1]
            self.Xb1[2,i] = self.odom_drone[2]
            self.Xb1[3,i] = self.odom_drone[3]

            Y = np.transpose([self.Xb1[0,i], self.Xb1[1,i], self.Xb1[2,i], self.Xb1[3,i]])

            Xtil = Xd - Y

            """self.erro[0,i] = Xtil[0,0]
            self.erro[1,i] = Xtil[1,0]
            self.erro[2,i] = Xtil[2,0]"""

            U = self.controlePosicionamento(X=Y, Xd=Xd, Xtil=Xtil, j=i, phi=self.phi)
            
            vel_msg.linear.x = U[i][0][0]
            vel_msg.linear.y = U[i][1][0]
            vel_msg.linear.z = U[i][2][0]
            vel_msg.angular.z = U[i][3][0]

            if not self.rospy.is_shutdown():                
                self.pubVel2.publish(vel_msg)
            self.rate.sleep()
        self.rospy.loginfo("Feito.")

    def goToInitialPositionDrone1(self, bebop_name, xd=None, yd=None, zd=None, phi=0):

        """
        Faz o drone ir para a posição inicial

        Parametros:
        bebop_name = Nome do drone
        xd,yd,zd,phi = posição e orientação do drone

        """
        self.U[0] = [[self.odom_drone[0] , self.odom_drone[1], self.odom_drone[2], self.odom_drone[3]]]
        Xd = np.transpose([[xd, yd, zd, phi]])
        self.rospy.loginfo("Levando o %s para a posição inicial..."%(bebop_name))
        vel_msg = Twist()
        for i in range(1,len(self.t)-1):
            X = np.transpose([[self.odom_drone[0] , self.odom_drone[1], self.odom_drone[2], self.odom_drone[3]]])

            #X = np.transpose([[self.x[i] , self.y[i], self.z[i], self.phi[i]]])

            Xtil = Xd - X
            U = self.controlePosicionamento(X=X, Xd=Xd, Xtil=Xtil, j=i, phi=self.phi)
            
            vel_msg.linear.x = U[i][0][0]
            vel_msg.linear.y = U[i][1][0]
            vel_msg.linear.z = U[i][2][0]
            vel_msg.angular.z = U[i][3][0]

            if not self.rospy.is_shutdown():                
                self.pubVel.publish(vel_msg)
            self.rate.sleep()
        self.rospy.loginfo("Feito.")
    

    def run(self):        

        #a execução espera o comando de takeoff finalizar
       # self.takeOff(self.bebop1_name)
        #self.takeOff(self.bebop2_name)

        #self.rospy.sleep(2)
        #self.goToInitialPositionDrone2(bebop_name=self.bebop2_name, xd=-2, yd=1, zd=0.75, phi=0)
        #self.rospy.sleep(2)
        #self.goToInitialPositionDrone1(bebop_name=self.bebop1_name, xd=0, yd=0, zd=0.75, phi=0)
        #self.rospy.sleep(2)


        #print(self.t)
        vel_msg = Twist()
        vel_msg2 = Twist()
        
        self.rospy.loginfo("Iniciando a formação...")
        for i in range(1,len(self.t)-1):
            #X = np.transpose([[self.odom_drone[0] , self.odom_drone[1], self.odom_drone[2], self.odom_drone[3]]])
            





            """self.odomx[i] = self.odom_drone[0]
            self.odomy[i] = self.odom_drone[1]
            self.odomz[i] = self.odom_drone[2]"""
            #Y = np.transpose([[self.Xb1[0,i], self.Xb1[1,i], self.Xb1[2,i], self.Xb1[3,i]]])


            #X = np.transpose([[self.x[i] , self.y[i], self.z[i], self.phi[i]]])

            #Xtil = self.Xd - Y
            #self.erro[0,i] = Xtil[0,0]
            #self.erro[1,i] = Xtil[1,0]
            #self.erro[2,i] = Xtil[2,0]

            #U = self.controlePosicionamento(X=Y, Xd=self.Xd,Xtil=Xtil, j=i, phi=self.phi)
            
            #vel_msg.linear.x = U[i][0][0]
            #vel_msg.linear.y = U[i][1][0]
            #vel_msg.linear.z = U[i][2][0]
            #vel_msg.angular.z = U[i][3][0]

            #if not self.rospy.is_shutdown():                
            #    self.pubVel.publish(vel_msg)
            
            U2 = self.controleFormacao(j=i, odomx=self.odom_drone[0],odomy=self.odom_drone[1],odomz=self.odom_drone[2],odomphi=self.odom_drone[3],
            odomx2=self.odom_drone2[0],odomy2=self.odom_drone2[1],odomz2=self.odom_drone2[2], odomphi2 = self.odom_drone2[3])
            
            vel_msg2.linear.x = U2[0]
            vel_msg2.linear.y = U2[1]
            vel_msg2.linear.z = U2[2]
            vel_msg2.angular.z = U2[3]
            if not self.rospy.is_shutdown():                
                self.pubVel2.publish(vel_msg)
            self.rate.sleep()

        
        #mpl.rcParams['legend.fontsize'] = 10
        self.rospy.loginfo("Formação finalizada.")

        
        self.land(self.bebop1_name)
        self.land(self.bebop2_name)

        """erros = plt.figure(1)
        errosFormacao = plt.figure(2)
        ax1 = erros.add_subplot(111)
        ax2 = errosFormacao.add_subplot(111)

        ax1.plot(self.t[:-1], self.erro[0,:], label="Erro X")        
        ax1.plot(self.t[:-1], self.erro[1,:], label="Erro Y")        
        ax1.plot(self.t[:-1], self.erro[2,:], label="Erro Z")
        ax1.set_title("Erros de posicionamento")
        erros.show()
        
        ax2.plot(self.t[:-1], self.errosFormacao[0,:], label="Erro Form. X")        
        ax2.plot(self.t[:-1], self.errosFormacao[1,:], label="Erro Form. Y")        
        ax2.plot(self.t[:-1], self.errosFormacao[2,:], label="Erro Form. Z")
        ax2.set_title("Erros de Formação")
        errosFormacao.show()"""
        fig, ax = plt.subplots(2)
        fig.set_figheight(20)
        fig.set_figwidth(20)
        ax[0].plot(self.t[:-1], self.erro[0,:], label="Erro X", )        
        ax[0].plot(self.t[:-1], self.erro[1,:], label="Erro Y")        
        ax[0].plot(self.t[:-1], self.erro[2,:], label="Erro Z")
        ax[0].set_title("Erros de posicionamento")

        ax[1].plot(self.t[:-1], self.errosFormacao[0,:], '--', label="Erro rhof")        
        ax[1].plot(self.t[:-1], self.errosFormacao[1,:], '-*', label="Erro betaf")        
        ax[1].plot(self.t[:-1], self.errosFormacao[2,:], 'o-', label="Erro alphaf")
        ax[1].set_title("Erros de formação")

        ef = open('errosFormacao.txt', 'a+')
        epos = open('errosPosicao.txt', 'a+')
        for i in range(0, len(self.errosFormacao)):
            erroForma = "%s,%s,%s\n"%(self.errosFormacao[0,i],self.errosFormacao[1,i],self.errosFormacao[2,i])
            erroPosic = "%s,%s,%s\n"%(self.erro[0,i],self.erro[1,i],self.erro[2,i])
            ef.write(erroForma)
            epos.write(erroPosic)     

        ef.close()
        epos.close()
        fig.legend()
        
        plt.show()
                
    
if __name__ == '__main__':
    try:
        Controller = Controller()
    except rospy.ROSInterruptException:
        pass