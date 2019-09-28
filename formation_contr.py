#! /usr/bin/env python2
# -*- coding: utf-8 -*-
#from __future__ import unicode_literals
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

import matplotlib.pyplot as plt
import matplotlib as mpl
import message_filters
import numpy as np
import rospy
import time
import math
import tf


class Controller:
    def __init__(self):
        self.rospy=rospy
        self.rospy.init_node('listener',anonymous=True)
        self.rospy.loginfo("Iniciando o controle de formação") #" starting controle de formacao de drones"        
        self.initvariables() # inicializacao de variaveis
        self.initSubscribers() # topicos a serem subscritos por ex odometria
        self.initPublishers(self.bebop1_name, self.bebop2_name)
        self.change=False #label para indicar mudanca de variaveis        
        self.run()  # loop de controle

    def initSubscribers(self):
        """
        Inicia os Subscribers dos tópicos do ROS
        """
        self.subOdom = self.rospy.Subscriber('/%s/new_odom'%(self.bebop1_name), Odometry, self.get_drone1_odom)
        self.subOdom2 = self.rospy.Subscriber('/%s/new_odom'%(self.bebop2_name), Odometry, self.get_drone2_odom)
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

        self.bebop1_name = 'B1'
        self.bebop2_name = 'B2'
        self.rate = self.rospy.Rate(5)
        self.Xd = np.transpose([[2, 1, 1.5, 0]])   
                      
        self.ti = 0.2
        self.tfinal = 20
        self.t = np.arange(0, self.tfinal, self.ti)

        #Formação desejada
        self.qdes = np.transpose([2, 1, 1.5, 1, 0, 0])        
        
        self.qtil = np.zeros((6, len(self.t)))

        self.Xb1 = np.zeros((4,len(self.t)))
        self.Xb2 = np.zeros((4,len(self.t)))

        #ks
        self.k1 = 0.8417
        self.k2 = 0.18227
        self.k3 = 0.8354
        self.k4 = 0.17095
        self.k5 = 3.966
        self.k6 = 4.001
        self.k7 = 9.8524
        self.k8 = 4.7295    

        #kps
        self.kpx = 0.2
        self.kpy = 0.2
        self.kpz = 0.2
        self.kpphi = 0.2
        self.kdx = 0.2
        self.kdy = 0.2
        self.kdz = 0.2
        self.kdphi = 0.2
        #kp
        self.kp = np.diag([self.kpx, self.kpy, self.kpz, self.kpphi])
        self.kd = np.diag([self.kdx, self.kdy, self.kdz, self.kdphi])


        self.Ku = np.diag([self.k1, self.k3, self.k5, self.k7])
        self.Kv = np.diag([self.k2, self.k4, self.k6, self.k8])
        self.K = np.diag([0.8, 0.8, 0.8, 0.8])

        #Condições iniciais
        #Drone 1
        self.x = 0*np.ones((len(self.t), 1))
        self.y = 0*np.ones((len(self.t),1))
        self.z = np.ones((len(self.t),1))
        self.phi = np.zeros((len(self.t),1))
        self.U = {}      

        #(x, y, z, psi, vel_x, vel_y, velz)
        self.odom_drone1 = (0, 0, 0, 0, 0, 0 )      
        
        #(x, y, z, psi, vel_x, vel_y, velz)
        self.odom_drone2 = (0, -1, 0, 0, 0, 0,0)

        self.v = {}        
        self.erro = np.zeros((6,len(self.t)))
        self.errox = []
        self.erroy = []
        self.erroz = []
        self.errorhof = []
        self.errobetaf = []
        self.erroalphaf = []

        self.errosFormacao = np.zeros((6,len(self.t)-1))

        #Condições iniciais
        #Drone2

        #initial position Drone2
        self.Vd1A = np.transpose([0, 0, 0, 0])
        self.Vd2A = np.transpose([0, 0, 0, 0])

    def get_drone1_odom(self, msg):

        """
        Atualiza a odometria do drone 1
        Parametros: 
        msg = Mensagem recebida do tópico
        """
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

        vel_x = msg.twist.twist.linear.x
        vel_y = msg.twist.twist.linear.y
        vel_z = msg.twist.twist.linear.z
        velAngZ = msg.twist.twist.angular.z

        #atualiza o valor da odometria
        self.odom_drone1 = (x, y, z, psi, vel_x, vel_y, vel_z, velAngZ)
        return



    def get_drone2_odom(self,msg):
        """
        Atualiza a odometria do drone 2
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

        vel_x = msg.twist.twist.linear.x
        vel_y = msg.twist.twist.linear.y
        vel_z = msg.twist.twist.linear.z
        velAngZ = msg.twist.twist.angular.z

        #atualiza o valor da odometria
        self.odom_drone2 = (x, y, z, psi, vel_x, vel_y, vel_z, velAngZ)


    def takeOff(self, drone_name1, drone_name2):

        """
        Inicia o vôo do drone

        Parametros: 
        drone_name1 = Nome do drone 1
        drone_name2 = Nome do drone 2
        """

        pub1 = self.rospy.Publisher("/%s/takeoff"%(drone_name1), Empty, queue_size=10)    
        pub2 = self.rospy.Publisher("/%s/takeoff"%(drone_name2), Empty, queue_size=10)

        rate_10 = self.rospy.Rate(5) # 10hz
        self.rospy.loginfo("Taking Off %s ..."%(drone_name1))
        self.rospy.loginfo("Taking Off %s ..."%(drone_name2))
        for i in range(1,25):
            pub1.publish(Empty())
            pub2.publish(Empty())
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
        rate_10 = self.rospy.Rate(5) # 10hz
        self.rospy.loginfo("Landing %s ..."%(drone_name))
        for i in range(1,25):
            pub.publish(Empty())
            rate_10.sleep()
        self.rospy.loginfo("Feito.")
        return True

    def controleFormacao(self, j=None, t=None, odom_drone1=[], odom_drone2=[]):
        """
        Realiza o controle de formação

        Parâmetros:
        t = tempo de execução
        odom_drone1 = Odometria do drone 1
        odom_drone2 = Odometria do drone 2
        Retorna: controles do drone 1 e 2

        """

        odomx = odom_drone1[0]
        odomy = odom_drone1[1] 
        odomz = odom_drone1[2]
        odompsi = odom_drone1[3]
        vel_x1 = odom_drone1[4]
        vel_y1 = odom_drone1[5]
        vel_z1 = odom_drone1[6] 
        velAngZ1 = odom_drone1[7]

        odomx2 = odom_drone2[0] 
        odomy2 = odom_drone2[1]
        odomz2 = odom_drone2[2]
        odompsi2 = odom_drone2[3]
        vel_x2 = odom_drone2[4]
        vel_y2 = odom_drone2[5]
        vel_z2 = odom_drone2[6] 
        velAngZ2 = odom_drone2[7]

        #rhof
        rhof = math.sqrt((odomx2 - odomx)**2 + (odomy2 - odomy)**2 + (odomz2 - odomz)**2)
        alphaf =  math.atan2((odomy2 - odomy), (odomx2-odomx))
        betaf = math.atan2((odomz2 - odomz), math.sqrt((odomx2 - odomx)**2 + (odomy2 - odomy)**2))
        q = np.array([odomx, odomy, odomz, rhof, alphaf, betaf])
        qtil = self.qdes - np.transpose(q)

        self.errox.append(qtil[0,])
        self.erroy.append(qtil[1,])
        self.erroz.append(qtil[2,])

        self.errorhof.append(qtil[3,])
        self.erroalphaf.append(qtil[4,])
        self.errobetaf.append(qtil[5,])


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

        jacob = np.array([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [1, 0, 0, math.cos(alphaf)*math.cos(betaf), -rhof*math.sin(alphaf)*math.cos(betaf), -rhof*math.cos(alphaf)*math.sin(betaf)],
                [0, 1, 0, math.sin(alphaf)*math.cos(betaf), rhof*math.cos(alphaf)*math.cos(betaf),  -rhof*math.sin(alphaf)*math.sin(betaf)],
                [0, 0, 1, math.sin(betaf), 0, rhof*math.cos(betaf)]])


        xrefp = jacob.dot(qrefp)
        #Cinemática direta Drone 1
        F1 = np.array([
            [math.cos(odompsi), -math.sin(odompsi), 0, 0],
            [math.sin(odompsi), math.cos(odompsi), 0, 0],
            [0,     0,      1,      0],
            [0,     0,      0,      1]])
        #Cinemática direta Drone 2
        F2 = np.array([
            [math.cos(odompsi2), -math.sin(odompsi2), 0, 0],
            [math.sin(odompsi2), math.cos(odompsi2), 0, 0],
            [0,     0,      1,      0],
            [0,     0,      0,      1]])

        F1 = np.linalg.inv(F1)
        F2 = np.linalg.inv(F2)

        Xd1p = np.append(xrefp[:3,], 0)
        Xd2p = np.append(xrefp[3:,], 0)

        Vd1 = F1.dot(Xd1p)
        Vd2 = F2.dot(Xd2p)

        #Deriva a velocidade
        Vd1p = (Vd1 - self.Vd1A)/(self.rospy.get_time()-t)
        Vd2p = (Vd2 - self.Vd2A)/(self.rospy.get_time()-t)

        #Atualiza as velocidades anteriores
        self.Vd1A = Vd1
        self.Vd2A = Vd2

        vel_bp1 = F1.dot(np.transpose([vel_x1,vel_y1,vel_z1,0]))
        vel_bp2 = F2.dot(np.transpose([vel_x2,vel_y2,vel_z2,0]))

        Ku_inv = np.linalg.inv(self.Ku)
        Ud1 = (Ku_inv.dot((Vd1p+self.K.dot(Vd1-vel_bp1) + self.Kv.dot(vel_bp1))))
        Ud2 = (Ku_inv.dot((Vd2p+self.K.dot(Vd2-vel_bp2) + self.Kv.dot(vel_bp2))))

        return (Ud1, Ud2)

    def run(self):        
        
        self.takeOff(self.bebop1_name, self.bebop2_name)
        self.rospy.sleep(3)
        
        vel_msg = Twist()
        vel_msg2 = Twist()
        
        self.rospy.loginfo("Iniciando a formação...")
        self.rospy.sleep(2)

        T_MAX = 20
        T_CONTROL = 0.2
        t = self.rospy.get_time()
        t_incB = self.rospy.get_time()
        t_control = self.rospy.get_time()
        i = 0

        #Sincroniza com o drone 1
        self.rospy.wait_for_message('/%s/new_odom'%(self.bebop1_name), Odometry)
        
        while ((self.rospy.get_time()-t) < T_MAX):
            if ((self.rospy.get_time() - t_control) > T_CONTROL):            
                U = self.controleFormacao(j=i, t=t_incB, odom_drone1=self.odom_drone1, odom_drone2=self.odom_drone2)

                #Velocidade drone1
                vel_msg.linear.x = U[0][0]
                vel_msg.linear.y = U[0][1]
                vel_msg.linear.z = U[0][2]

                #Velocidade drone2
                vel_msg2.linear.x = U[1][0]
                vel_msg2.linear.y = U[1][1]
                vel_msg2.linear.z = U[1][2]

                if not self.rospy.is_shutdown():                
                    self.pubVel.publish(vel_msg)
                    self.pubVel2.publish(vel_msg2)
                t_control = self.rospy.get_time()
                t_incB = self.rospy.get_time()                
                i = i + 1        
        
        self.rospy.loginfo("Formação finalizada.")        
        self.land(self.bebop1_name)
        self.land(self.bebop2_name)

        """
        Plotagem dos erros de posição e formação
        """

        fig, ax = plt.subplots(2)
        fig.set_figheight(20)
        fig.set_figwidth(20)
        ax[0].plot(self.t[:i], self.errox[:], label="Erro X", )        
        ax[0].plot(self.t[:i], self.erroy[:], label="Erro Y")        
        ax[0].plot(self.t[:i], self.erroz[:], label="Erro Z")
        ax[0].set_title("Erros de posicionamento")

        ax[1].plot(self.t[:i], self.errorhof[:], '--', label="Erro rhof")        
        ax[1].plot(self.t[:i], self.errobetaf[:], '--', label="Erro betaf")        
        ax[1].plot(self.t[:i], self.erroalphaf[:], '--', label="Erro alphaf")
        ax[1].set_title("Erros de formacao")
        fig.legend()        
        plt.show()
                
    
if __name__ == '__main__':
    try:
        Controller = Controller()
    except rospy.ROSInterruptException:
        pass