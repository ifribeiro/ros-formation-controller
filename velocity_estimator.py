#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
import time
import math
from scipy import signal
from numpy import argmax, mean, diff, log, nonzero


class Estimator:
    def __init__(self):
        self.rospy=rospy
        self.rospy.init_node('listener',anonymous=True)
        self.rospy.loginfo("Starting Leg Laser") #" starting controle de formacao de drones"
        # self.initParameters()
        self.initSubscribers() # topicos a serem subscritos por ex odometria
        self.initvariables() # inicializacao de variaveis
        self.change=False #label para indicar mudanca de variaveis
        self.run()  # loop de controle

    def initSubscribers(self):
        self.subPose = self.rospy.Subscriber("/scan", LaserScan, self.getlaser) # topico subscrito --> tipo de mensagem que o topico manda -->
        # --> metodo que vai atualizar dados da mensagem para atributos do objeto
        self.subvel = self.rospy.Subscriber("/RosAria/pose", Odometry, self.get_robot_velocity)
        return
    def initvariables(self):
        self.rate = self.rospy.Rate(100)
        self.angle_min = 0
        self.angle_max = 0
        self.scan_time = 0
        self.ranges = 0
        self.angle_increment = 0
        self.time_increment = 0
        self.range_min = 0
        self.range_max = 0
        self.change = False
        self.robot_linear_vel=0
        #wflc

        self.omega_0_init = 1

        # self.sum_rec = 0
        # self.sum_omega_0 = 0
        # self.harmonics = 0
        # self.tremor = 0            
        self.mu_0 = 2*10**(-6)
        self.mu_1 =  1.5*10**(-3)
        self.mu_b = 0
        self.M=1
        #fle
        self.mu=0.0018
        self.sum_omega=0
        self.harmonics_flc=0
        self.noise_estimate_flc=np.zeros(2)
        
        self.count=0
        self.count_vec=[]
        self.R_leg_pose=[]
        self.L_leg_pose=[]
        self.LDD_vector=[]
        # self.Gait_Candence_vector=[]self.angle_min = 0
        self.angle_max = 0
        self.scan_time = 0
        self.ranges = 0
        self.Gait_Amplitude_vector=[]
        self.angle_min = 0
        self.angle_max = 0
        self.scan_time = 0
        self.ranges = 0
        self.angle_min = 0
        self.angle_max = 0
        self.scan_time = 0
        self.ranges = 0
        self.Estimated_Human_Linear_Velocity=[]
        self.Robot_Linear_velocity=[]
        self.Gait_Candence_vector=[]

    def get_robot_velocity(self,msg):
        robot_linear_vel = msg
        self.robot_linear_vel=robot_linear_vel.twist.twist.linear.x
        return
        
    def getlaser(self,msg):
        self.angle_min = msg.angle_min
        self.angle_max =msg.angle_max
        self.time_increment = msg.time_increment
        self.angle_increment= msg.angle_increment
        self.scan_time = msg.scan_time 
        self.range_min =msg.range_min 
        self.range_max =msg.range_max 
        self.ranges=msg.ranges
        self.intensities = msg.intensities
        self.change=True


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
                
    def wflc3(self,y,mu_0,mu_1,mu_b,omega_0_init,M):
        # print(len(y))
        T=1/100
        sum_omega_0=0
        X=np.zeros((2*M,len(y)))
        W=np.zeros((2*M,len(y)))
        omega_0=np.zeros(len(y))
        omega_0[0]=omega_0_init*2*math.pi*T
        
        for k in range(0,len(y)):
            
            X[0,k]=math.sin(sum_omega_0)
            X[1,k]=math.cos(sum_omega_0)
            error = y[k]-np.transpose(W[:,k]).dot(X[:,k]) - mu_b
            sum_rec =0
            sum_rec=sum_rec + (W[0,k]*X[1,k] - W[1,k]*X[0,k])

            omega_0_pred = omega_0[k] + 2*mu_0*error*sum_rec

            W_pred = W[:,k]+2*mu_1*error*X[:,k]

            if(k < (len(y)-1)):
                #print('entrei')
                omega_0[k+1]=omega_0_pred
                W[:,k+1]=W_pred
                sum_omega_0=sum_omega_0+omega_0_pred
                #print(omega_0[k])
            else:
                break
        #filter outputs
        omega_0_Hz= omega_0/(2*math.pi)/T
        
        #harmonics = np.multiply(W[0:2*M,:],X[0:2*M,:])
        #tremor = np.sum(harmonics[0:2*M,:])
        self.Gait_Candence_vector=omega_0_Hz
            #return omega_0_Hz,harmonics,tremor

    def flc(self,y,mu,freq2,M):
        T=1/100
        omega = 2*math.pi*T*(freq2)
        
        X=np.zeros((2*M,len(y)))
        W=np.zeros((2*M,len(y)))
        sum_omega=0
        for k in range(0,len(y)):
            X[0,k]=math.sin(sum_omega)
            X[1,k]=math.cos(sum_omega)
            error = y[k]-np.dot(np.transpose(W[:,k]),X[:,k])
            W_pred=W[:,k]+2*mu*error*X[:,k]
            if(k<(len(y)-1)):
                W[:,k+1]=W_pred
                sum_omega=sum_omega+omega[k]
            else:
                break
        harmonics = (np.multiply(W[0:2*M,:],X[0:2*M,:]))
        #print(harmonics)
        noise_estimate = np.sum(harmonics[0:2*M,:])
        #print(noise_estimate)
        for k in range(0,len(y)):
            self.Gait_Amplitude_vector.append(100*math.sqrt(harmonics[0,k]**2+harmonics[1,k]**2))
                    #print(self.Gait_Amplitude_vector)  


        return  # each amplitude multiplied
        #by its sinusoid

    def legsCluster(self):
            
            n1=0
            x_n1=0
            y_n1=0
            c1=np.zeros(2,dtype=float)
            n2=0
            x_n2=0
            y_n2=0
            c2=np.zeros(2,dtype=float)
            clusters = KMeans(n_clusters=2).fit(self.cart)
            labels = clusters.labels_
            
            n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
            if(n_clusters != 0):
                for i in range(0,len(labels)):
                    if(labels[i]==0):
                        n1=n1+1
                        x_n1=x_n1+self.cart[i,0]
                        y_n1=y_n1+self.cart[i,1]
                    else:
                        n2=n2+1
                        x_n2=x_n2+self.cart[i,0]
                        y_n2=y_n2+self.cart[i,1]
                
                if(y_n1>y_n2):
                    c1=[x_n1/n1,y_n1/n1]
                    c2=[x_n2/n2,y_n2/n2]
                else:
                    c2=[x_n1/n1,y_n1/n1]
                    c1=[x_n2/n2,y_n2/n2]

            self.R_leg_pose.append(c1[0]*1000)
            self.L_leg_pose.append(c2[0]*1000)

            #plt.plot(c1[0],self.start)
            self.LDD = (c1[0]-c2[0])*1000  
            self.LDD_vector.append(self.LDD)   
                #LDD=math.sqrt(pow(c1[0]-c2[0],2)+pow(c1[1]-c2[1],2))
                  

            # plt.scatter(self.cart[:,0],self.cart[:,1],c=labels.astype(np.float), edgecolor='k')
            # #plt.plot([c1[0],c2[0]],[c1[1],c2[1]],'-')
            # plt.text(c1[0],c1[1], "LEG 1")
            # plt.text(c2[0],c2[1], "LEG 2")
            # plt.text(0,0,"LDD = {} mm".format(round(LDD,3)))
            # #plt.plot(np.arange(0,1),1)


            # print(c1)
            # print(c2)
            # print(self.cart.shape)
            # print("----")

            self.fig.canvas.draw()
           

            return 



    def run(self):     
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
                
        self.rate.sleep()
    
if __name__ == '__main__':
    try:
        Estimator = Estimator()
    except rospy.ROSInterruptException:
        pass

