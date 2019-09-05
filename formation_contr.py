#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D

import math
import numpy as np
import matplotlib.pyplot as plt
import time
import math
#from scipy import signal
from numpy import argmax, mean, diff, log, nonzero


class Controller:
    def __init__(self):
        self.rospy=rospy
        self.rospy.init_node('listener',anonymous=True)
        self.rospy.loginfo("Starting Formation Controller") #" starting controle de formacao de drones"
        # self.initParameters()        
        self.initvariables() # inicializacao de variaveis
        self.initSubscribers() # topicos a serem subscritos por ex odometria
        self.initPublishers(self.bebop1_name)
        self.change=False #label para indicar mudanca de variaveis        
        self.run()  # loop de controle

    def initSubscribers(self):
        # --> metodo que vai atualizar dados da mensagem para atributos do objeto
        #self.subvel = self.rospy.Subscriber("/turtle1/pose", Pose, self.get_robot_velocity)
        #self.subOdom = self.rospy.Subscriber('/%s/odom'%(self.bebop1_name), Odometry, self.get_drone_odom)
        self.subTurtle = self.rospy.Subscriber('/%s/pose'%("turtle1"), Pose, self.get_turtle_odom)
        return
    def initPublishers(self, bebop1_name):
        #self.pubVel = rospy.Publisher('/%s/cmd_vel'%(bebop1_name), Twist, queue_size=10)

        self.pubVel2 = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

        return 
    def initTimeArray(self, start_time, end_time):
        np.range()

    def initvariables(self):
        self.bebop1_name = 'bebop1'
        self.rate = self.rospy.Rate(200)
        #self.Xd = np.transpose([[2, 1, 1.5, 0]])   
        self.Xd = np.transpose([[8, 5.54, 0, 0]])
        
        self.ti = 0.1
        self.tfinal = 10
        self.t = np.arange(0, self.tfinal, self.ti)

        #Formação desejada
        #self.qdes = np.transpose([[2, 1, 1.5, 1, 0, 0]])
        self.qdes = np.transpose([[8, 5.54, 0, 1, 0, 0]])
        
        self.qtil = np.zeros((6, len(self.t)))

        #ks
        self.k1 = 5
        self.k2 = 0.3
        self.k3 = 5.5
        self.k4 = 0.2
        self.k5 = 0.55
        self.k6 = 0.7
        self.k7 = 1.45
        self.k8 = 0.7        

        #kps
        self.kpx = 3
        self.kpy = 3
        self.kpz = 0.5
        self.kpphi = 3
        self.kdx = 2
        self.kdy = 2.5
        self.kdz = 2
        self.kdphi = 0.5
        #kp
        self.kp = np.diag([self.kpx, self.kpy, self.kpz, self.kpphi])
        self.kd = np.diag([self.kdx, self.kdy, self.kdz, self.kdphi])

        #Condições iniciais
        self.x = 5.54*np.ones((len(self.t), 1))
        self.y = 5.54*np.ones((len(self.t),1))
        self.z = 0*np.ones((len(self.t),1))
        self.phi = np.zeros((len(self.t),1))
        self.U = {}
        self.ganho = 0.5

        self.odom_drone = (5.54, 5.54, 0, 0)
        self.old_odom_drone = (0,0,0,0)

        self.a=0.2
        self.v = {}
        self.odomx = 5.54*np.ones(len(self.t)-1)
        self.odomy = 5.54*np.zeros(len(self.t)-1)


    def get_robot_velocity(self,msg):
        robot_linear_vel = msg
        self.robot_linear_vel = (robot_linear_vel.x, robot_linear_vel.y, robot_linear_vel.theta)
        return
    def get_drone_odom(self, msg):
        
        self.old_odom_drone = self.odom_drone

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        psi = msg.twist.twist.angular.z

        #atualiza o valor da odometria
        self.odom_drone = (x, y, z, psi)

        #verifica se a odometria mudou
        if(self.old_odom_drone != self.odom_drone):
            self.change = True
        return

    def get_turtle_odom(self, msg):
        x = msg.x
        y = msg.y
        self.odom_drone = (x, y, 0, 0.0)

    def takeOff(self, drone_name):
        pub = self.rospy.Publisher("/%s/takeoff"%(drone_name), Empty, queue_size=10)    
        rate_10 = self.rospy.Rate(10) # 10hz
        for i in range(1,25):
            pub.publish(Empty())
            rate_10.sleep()
        return True

    def run(self):        

        #a execução espera o comando de takeoff finalizar
        #self.takeOff(self.bebop1_name)  
        #print(self.t)
        vel_msg = Twist()

        for i in range(0,len(self.t)-1):
            X = np.transpose([[self.odom_drone[0] , self.odom_drone[1], self.odom_drone[2], self.odom_drone[3]]])
            self.odomx[i] = self.odom_drone[0]
            self.odomy[i] = self.odom_drone[1]
            #X = np.transpose([[self.x[i] , self.y[i], self.z[i], self.phi[i]]])
            Xtil = self.Xd - X
            f1 = [
                [(self.k1)*(math.cos(self.phi[i])),(-self.k3)*(math.sin(self.phi[i])), 0, 0],
                [(self.k1)*math.sin(self.phi[i]), self.k3*math.cos(self.phi[i]), 0, 0],
                [0, 0, self.k5, 0],
                [0, 0, 0, self.k7]]
            f1_inv = np.linalg.inv(f1)
            #print ("f1-1", f1_inv)
            xd_x_to = self.Xd*self.ti
            #print ("xd*xto \n", xd_x_to)

            kd_Xtil = self.kd.dot(Xtil)

            #print ("kd_Xtil \n", kd_Xtil)

            tanh_kd_Xtil = np.transpose([np.array([math.tanh(xi) for xi in kd_Xtil])])
            #print("tanh_kd_Xtil \n",tanh_kd_Xtil)
                    
            part3 = self.kp.dot(tanh_kd_Xtil)

            #print("kp*tanh_kd_til \n", part3)
            
            xd_kp = xd_x_to + part3
            #print ("part1 \n", f1_inv_x_xd.diagonal()) 
            #print ("xd_x_to + part3 \n", xd_kp)
            #f1_inv_x_xd = (f1_inv).dot(xd_kp)

            #print ("f1_inv_x_xd \n", f1_inv_x_xd) 
            self.U[i] = ((f1_inv).dot(xd_kp))*self.ganho

            vel_msg.linear.x = self.U[i][0][0]
            vel_msg.linear.y = self.U[i][1][0]
            vel_msg.linear.z = self.U[i][2][0]
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            if not self.rospy.is_shutdown():                
                self.pubVel2.publish(vel_msg)
            time.sleep(self.ti)
            
            #print (self.U[i][0][0], self.U[i][1][0], self.U[i][2][0])
            #Define a nova posicão do drone1
            self.x[i+1] = self.x[i]+self.ti*self.U[i][0][0]
            self.y[i+1] = self.y[i]+self.ti*self.U[i][1][0]
            self.z[i+1] = self.z[i]+self.ti*self.U[i][2][0]
            self.phi[i+1] = self.phi[i]+self.ti*self.U[i][3][0]
            
            #time.sleep(self.ti)

            xf = self.x[i][0]
            yf = self.y[i][0]
            zf = self.z[i][0]
    
            #Distancia entre os drones
            rhof = 0
            #alfaf
            alphaf = 0
    
            #betaf
            betaf = 0
            #q
            q = [xf, yf, zf, rhof, betaf, alphaf]

            
            
            self.qtil[:,i] = (self.qdes - np.transpose(q))[:,0]       
            

            #Ganhos
            L1 = 0.2*np.identity(6)
            L2 = 0.3*np.identity(6)            
            L2_x_qtil = L2.dot(self.qtil[:,i])
            tanh_L2qtil = [math.tanh(li) for li in L2_x_qtil]         
            qrefp = L1.dot(tanh_L2qtil) 
            jacob =np.array([[1, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 0, 1, 0, 0, 0],
                    [1, 0, 0, math.cos(alphaf)*math.cos(betaf), -rhof*math.sin(alphaf)*math.cos(betaf), -rhof*math.cos(alphaf)*math.sin(betaf)],
                    [0, 1, 0, math.sin(alphaf)*math.cos(betaf), -rhof*math.sin(alphaf)*math.sin(betaf),  -rhof*math.cos(alphaf)*math.cos(betaf)],
                    [0, 0, 1, math.sin(betaf), rhof*math.cos(betaf), 0]])

            xrefp = jacob.dot(qrefp)

            K = np.array([  [math.cos(self.z[i]), math.sin(self.z[i]), 0, 0, 0, 0],
                            [-math.sin(self.z[i])/self.a, math.cos(self.z[i])/self.a, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, math.cos(self.phi[i]), -math.sin(self.phi[i]), 0],
                            [0, 0, 0, math.sin(self.phi[i]), math.cos(self.phi[i]), 0],
                            [0, 0, 0, 0, 0, 1]])

            self.v[i] = K.dot(xrefp)
            """
            vel_msg.linear.x = self.v[i][0]
            vel_msg.linear.y = self.v[i][1]
            vel_msg.linear.z = self.v[i][2]
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0"""

            """if not self.rospy.is_shutdown():                
                self.pubVel2.publish(vel_msg)
            time.sleep(self.ti)"""

            

            """%Ganhos
            xrefp = jacob*qrefp;
            
            K = [cos(pos(3,i)) sin(pos(3,i)) 0 0 0 0; ...
                -sin(pos(3,i))/a cos(pos(3,i))/a 0 0 0 0; ...
                0 0 1 0 0 0; ...
                0 0 0 cos(phid) -sin(phid) 0; ...
                0 0 0 sin(phid) cos(phid) 0; ...
                0 0 0 0 0 1];
            
            v{i} = K*xrefp; 
            """
            #print (self.x[i+1], ",", self.y[i+1], ",", self.z[i+1], self.phi[i+1])       

            """
            u = [ Uzponto Upsi Uphi Utheta ]
            Uzponto = velocidade linear sobre o eixo z

            Upsi = comando de velocidade angular em torno do eixo Z

            Uphi (UVy) = comando de inclinação em relação a xW (eixo yB do drone)

            Utheta (UVx) = comando de inclinação em relação ao eixo yW (eixo xB do drone)

            """
       
        #mpl.rcParams['legend.fontsize'] = 10
        print (self.odomx)
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        #ax = Axes3D(plt.gcf())
        ax.plot(self.odomx, self.odomy, label='Experimento')
        ax.plot(self.x[:,0],self.y[:,0],label='Simulação')
        plt.legend()
        
        plt.show()
        
    



        """
        while not self.r ospy.is_shutdown():
            if (self.change):
                print (self.x)
            
            self.change=False
            self.rate.sleep()
            break"""

        """
        self.fig = plt.figure(figsize=(18,10))
        
        while not self.rospy.is_shutdown():
            
                    self.fig.show() 
                self.change=False
                  


            
               # plt.clf()
        """
                
    
if __name__ == '__main__':
    try:
        Controller = Controller()
    except rospy.ROSInterruptException:
        pass