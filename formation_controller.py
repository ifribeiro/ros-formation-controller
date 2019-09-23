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
import message_filters
#from scipy import signal
from numpy import argmax, mean, diff, log, nonzero


class Estimator:
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
        self.subOdom = message_filters.Subscriber("/turtle1/pose", Pose)
        self.cache = message_filters.Cache(self.subOdom, 5, allow_headerless=True)
        #self.cache.registerCallback(self.callback)

        return
    
    def callback(self, data):
        pose = data
        x = pose.x
        y = pose.y 
        self.pose = (x,y)

    def initPublishers(self, bebop1_name):
        self.pubVel = rospy.Publisher('/%s/cmd_vel'%(bebop1_name), Twist, queue_size=10)
        return 

    def initvariables(self):
        self.bebop1_name = 'bebop1'
        self.rate = self.rospy.Rate(200)
        self.Xd = [5,5,5,0]
        self.x = [0,0,0,0]
        self.angle_min = 0
        self.angle_max = 0
        self.scan_time = 0
        self.ranges = 0
        self.angle_increment = 0
        self.time_increment = 0
        self.range_min = 0
        self.range_max = 0
        self.change = False
        self.altitudeChanged = False
        self.robot_linear_vel=0
        #wflc
        self.pose = (0,0)

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

    # def wflc(self,y, mu_0, mu_1, mu_b,omega_0, M):
    #     self.X[0]=math.sin(M*self.sum_omega_0)
    #     self.X[1]=math.cos(M*self.sum_omega_0)
        
    #     error=y-(M*(np.dot(self.W.T, self.X)))-mu_b
    #     #print(error)
    #     self.sum_rec=self.sum_rec+M*(self.W[0]*self.X[M])-(self.W[M]*self.X[0])
    #    # print(self.sum_rec)

    #     omega_0_pred = self.omega_0 + (2*mu_0*error*self.sum_rec)
    #     w_pred = self.W + (2*mu_1*error*self.X)

    #     self.omega_0 = omega_0_pred
    #     self.W = w_pred
    #     self.sum_omega_0 = self.sum_omega_0+omega_0_pred

    #     #outputs
    #     self.Omega_0_Hz = self.omega_0/(2*math.pi)/self.T
    #     self.harmonics_wlfc = np.multiply(self.W,self.X)
    #     self.tremor_wflc = np.sum(self.harmonics_wlfc)

    #     self.Gait_Candence_vector.append(self.Omega_0_Hz)

    #     return 
    def takeOff(self, drone_name):
        pub = self.rospy.Publisher("/%s/takeoff"%(drone_name), Empty, queue_size=10)    
        rate_10 = self.rospy.Rate(10) # 10hz
        for i in range(1,25):
            pub.publish(Empty())
            rate_10.sleep()
        return True
    def element(self,msg):
        print (msg)

    def run(self):
        T_MAX = 20
        T_CONTROL = 0.2

        t = self.rospy.get_time()
        t_control = self.rospy.get_time()
        i = 0       

        while ((self.rospy.get_time()-t) < T_MAX):
            if ((self.rospy.get_time() - t_control)>T_CONTROL):
                t_control = self.rospy.get_time()
                
                #self.element(self.cache.getElemBeforeTime(self.rospy.Time.now()))
                stamp = self.cache.getLastestTime()
                self.element(self.cache.getElemBeforeTime(stamp))
                i = i+1
                
        
        

        """
        while not self.rospy.is_shutdown():
            self.pubVel.publish(vel_msg)
            if (self.change):
                print (self.x)
            
            self.change=False
            self.rate.sleep()"""

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
        Estimator = Estimator()
    except rospy.ROSInterruptException:
        pass

