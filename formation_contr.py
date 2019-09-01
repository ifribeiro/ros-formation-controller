#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
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
        self.subOdom = self.rospy.Subscriber('/%s/odom'%(self.bebop1_name), Odometry, self.get_drone_odom)
        return
    def initPublishers(self, bebop1_name):
        self.pubVel = rospy.Publisher('/%s/cmd_vel'%(bebop1_name), Twist, queue_size=10)
        return 
    def initTimeArray(self, start_time, end_time):
        np.range()

    def initvariables(self):
        self.bebop1_name = 'bebop1'
        self.rate = self.rospy.Rate(200)
        self.Xd = np.transpose([[5, 5, 5, 0]])    
        self.ti = 0.1
        self.t = np.arange(0, 20, self.ti)

        #Formação desejada
        self.qdes = np.transpose([[2, 1, 1.5, 1, 0, 0]])

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
        self.x = -2*np.ones((len(self.t), 1))
        self.y = np.ones((len(self.t),1))
        self.z = np.ones((len(self.t),1))
        self.phi = np.zeros((len(self.t),1))
        self.U = {}
        self.ganho = 0.4

    def get_robot_velocity(self,msg):
        robot_linear_vel = msg
        self.robot_linear_vel = (robot_linear_vel.x, robot_linear_vel.y, robot_linear_vel.theta)
        return
    def get_drone_odom(self, msg):

        x = msg.pose.pose.position.x
        z = msg.pose.pose.position.z
        self.x[0] = x
        self.x[2] = z
        self.change=True
        return
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

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0.2
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        for i in range(1,len(self.t)):
            X = np.transpose([[self.x[i], self.y[i], self.z[i], self.phi[i]]])
            Xtil = self.Xd-X
            f1 = [
                [(self.k1)*(math.cos(self.phi[i])),(-self.k3)*(math.sin(self.phi[i])), 0, 0],
                [(self.k1)*math.sin(self.phi[i]), self.k3*math.cos(self.phi[i]), 0, 0],
                [0, 0, self.k5, 0],
                [0, 0, 0, self.k7]]
            part1 = np.linalg.inv(f1)
            part2 = self.Xd*self.ti
            part5 = self.kd.dot(Xtil)
            tanhs = np.transpose([np.array([math.tanh(xi) for xi in part5])])
            print ("->>>>")        
            part3 = self.kp.dot(tanhs)
            part4 = part2+part3

            self.U[i] = ((part1).dot(part2 + part3))*self.ganho

            print (self.U[1][1][0])
            break
            print("teste")
            




        while not self.rospy.is_shutdown():
            self.pubVel.publish(vel_msg)
            if (self.change):
                print (self.x)
            
            self.change=False
            self.rate.sleep()
            break

        """
        self.fig = plt.figure(figsize=(18,10))
        
        while not self.rospy.is_shutdown():
            self.msg=LaserScan()
            self.RosAria_msg=Odometry()
            self.x_cartesian=np.zeros(256,dtype=float)
            self.y_cartesian=np.zeros(256,dtype=float)
            self.cartesian = [ ]
            self.start = time.time()
            if(self.change):
                for i in range(0,len(self.ranges)):
                    #separando amostras por range de angulo (45graus) e por distancia do laser(0.5m)
                    if(i>=128 and i<=383 and self.ranges[i]<0.7): #separando amostras por range de angulo
                        self.x_cartesian[i-128]=self.ranges[i]*(math.cos(self.angle_min+(self.angle_increment*i)))
                        self.y_cartesian[i-128]=self.ranges[i]*(math.sin(self.angle_min+(self.angle_increment*i)))
                        if (self.y_cartesian[i-128]<-0.04 or self.y_cartesian[i-128]>0.04): # limiar de distancia entre as pernas
                            self.cartesian.append([self.x_cartesian[i-128],self.y_cartesian[i-128]])                        
                self.cart= np.array(self.cartesian)
                if self.cartesian != []:
                   self.count_vec.append(self.count*0.1)
                   self.count=self.count+1
                   self.legsCluster() 
                   self.Robot_Linear_velocity.append(self.robot_linear_vel*1000)
                   
                if(len(self.count_vec)==200):
                    sos = signal.butter(2, 0.2, 'hp', fs=100, output='sos')
                    self.Y = signal.sosfilt(sos, self.LDD_vector)

                    # self.b,self.a=signal.butter(2,0.2,'highpass')
                    # self.Y=signal.lfilter(b,a,self.LDD_vector)
                    self.wflc3(self.Y,self.mu_0,self.mu_1,self.mu_b,self.omega_0_init,self.M)
                    self.flc(self.LDD_vector,self.mu,self.Gait_Candence_vector,self.M)

                    self.Estimated_Human_Linear_Velocity=np.multiply(self.Gait_Candence_vector,self.Gait_Amplitude_vector)/100
                    #print(self.LDD_vector) 
                    
                    self.ax1=plt.subplot2grid((5,1),(0,0))
                    plt.plot(self.count_vec,self.R_leg_pose,'b-',self.count_vec,self.L_leg_pose,'r--')
                    
                    plt.xticks(np.arange(0,25,5))
                    plt.yticks(np.arange(0,1500,500))
                    plt.legend(['Perna Direita','Perna Esquerda'],loc='best',fontsize = 'x-small')
                    
                    plt.ylabel('[mm]')
                    plt.grid(True)

                    self.ax2=plt.subplot2grid((5,1),(1,0))
                    plt.plot(self.count_vec,self.LDD_vector,'b-')
                    plt.xticks(np.arange(0,22,5))
                    plt.yticks(np.arange(-300,600,300))
                    plt.ylabel('[mm]')
                    plt.grid(True)
                    
                    #print(self.Gait_Candence_vector)
                    self.ax3=plt.subplot2grid((5,1),(2,0))
                    plt.plot(self.count_vec,self.Gait_Candence_vector,'b-')
                    plt.xticks(np.arange(0,25,5))
                    plt.yticks(np.arange(0,3,1 ))
                    plt.ylabel('[passos/s]')
                    plt.grid(True)

                    self.ax4=plt.subplot2grid((5,1),(3,0))
                    plt.plot(self.count_vec,self.Gait_Amplitude_vector,'b-')
                    #plt.xticks(np.arange(0,25,5))
                    plt.ylabel('[mm]')
                    #plt.yticks(np.arange(0,12.5,2.5))
                    plt.grid(True)

                    self.ax3=plt.subplot2grid((5,1),(4,0))
                    plt.plot(self.count_vec,self.Estimated_Human_Linear_Velocity*100,'b-',self.count_vec,self.Robot_Linear_velocity,'--')
                    plt.xticks(np.arange(0,25,5))
                    plt.yticks(np.arange(0,800,200))
                    plt.legend(['Vhumano','Vrobô'],loc='best',fontsize = 'x-small')
                    plt.xlabel('[s]')
                    plt.ylabel('[mm/s]')
                    #plt.yticks(np.arange(0,12.5,2.5))
                    plt.grid(True)
                


                    #plt.tight_layout()
                    #self.fig.savefig('graphs.png')
                    self.fig.show() 
                self.change=False
                  


            
               # plt.clf()
        """
                
    
if __name__ == '__main__':
    try:
        Controller = Controller()
    except rospy.ROSInterruptException:
        pass