import Pyro.core
from hydra_motors_stepper import HydraMotor
import numpy as np
import flydra_for_hydra_trax as ffh
import threading
import socket
import flydra
import time
import Queue
import hydra_cal
import hydra_motors_stepper as h
import copy
import scipy.linalg as linalg
import sys
import os
import scipy
pi = np.pi


class Trax:

    

    def __init__(self,motorid):
        self.motorid = motorid
        self.pos = Queue.Queue()

        
        print 'initializing flydra'
        self.initialize_flydra()
        print 'done initializing flydra'
        
        self.old_data = [0,0,0,0,0,0]
        self.most_recent_data = None
        
        self.m_old_vel = 0
        self.looptime_i = 0
        
        self.direction = 0
        
        self.m = h.HydraMotor(self.motorid)
        
        

    def initialize_flydra(self):
        
            # set up flydra
		    connect_to_mainbrain = 1

		    if connect_to_mainbrain:
			    # make connection to flydra mainbrain
			    my_host = '' # get fully qualified hostname
			    my_port = 8320 + self.motorid # arbitrary number

			    # create UDP socket object, grab the port
			    sockobj = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			    print 'binding',( my_host, my_port)
			    sockobj.bind(( my_host, my_port))

			    # connect to mainbrain
			    self.mainbrain_hostname = 'sericomyia'
			    self.mainbrain_port = flydra.common_variables.mainbrain_port
			    self.mainbrain_name = 'main_brain'
			    remote_URI = "PYROLOC://%s:%d/%s" % (self.mainbrain_hostname,
				                             self.mainbrain_port,
				                             self.mainbrain_name)
			    Pyro.core.initClient(banner=0)
			    self.mainbrain = Pyro.core.getProxyForURI(remote_URI)
			    self.mainbrain._setOneway(['log_message'])
			    my_host_fqdn = socket.getfqdn(my_host)
			    self.mainbrain.register_downstream_kalman_host(my_host_fqdn,my_port)

                            print 'creating listener with sockobj',sockobj
			    self.listener = ffh.Listener(sockobj, dummy=False)

		    else:
			    self.listener = ffh.Listener(None,dummy=True)
			    self.mainbrain = ffh.DummyMainbrain()
		    listen_thread = threading.Thread(target=self.listener.run)
		    listen_thread.setDaemon(True)
		    listen_thread.start()
		    self.listener.push_fly_pos(self) 
		    
    def set_voi(self, center = [0,0,0], sides = [0,0,0], post_center = [0,0,0], post_diam = 0, post_buff = 0, speed = 0):
        self.listener.set_voi(center, sides, post_center, post_diam, post_buff, speed)
		    
    def initmotor(self):
        
        self.pantiltcamera = hydra_cal.PanTiltCamera()
    
        self.Mhat = self.pantiltcamera.Mhat
        #self.motor_adjust, self.fit = hydra_cal.getFocus()
        #self.T = np.matrix(hydra_cal.getw2c(self.Mhat))
        
        self.Mhat = np.matrix(self.Mhat)
        
        self.h1_integral_err = 0


        
        # zero
        if 1:
            print 'zeroing motors'
            if self.motorid != 3:
                self.m.findzeros()
                print 'done zeroing motors'
            if self.motorid == 3:
                self.m.infinity(5)
                print 'done zeroing motor 3'
                
                # open port to listen to gui
                self.recvsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                my_host = '' # get fully qualified hostname
                my_port = 30045 # arbitrary number
                self.recvsock.setblocking(0)
                self.recvsock.bind(( my_host, my_port))
                
                self.motor_offset = 0
                
        
        time.sleep(5)
        
    def followpos(self, fly_tmp):

        motorid = self.motorid
        self.looptime_i = time.time()
           
    
        fly_pos = np.matrix(copy.copy(fly_tmp)[0:3])
        fly_vel = np.matrix(copy.copy(fly_tmp)[3:6])
        latency = copy.copy(fly_tmp)[6]
        
        if latency > 0.1 or latency <0.00001:
            latency = 0.05
            
        #print latency
        fly_pos_est = fly_pos + fly_vel*latency
        fly_pos = np.concatenate((fly_pos_est,np.matrix(1)),1)
        #print fly_pos
        
        
        
        # where is motor, what is its velocity?
        m_current_pos = self.m.getpos()
        m_current_vel = self.m.getvel()
        
        damping = 0.02
        gain = 15
        
    
        if 1:
    
            change_direction = 1
    
            if motorid == 1:

                m_des_pos = self.pantiltcamera.get_motor_pos(fly_pos)[0]
            
                # controller:
                vel_des = gain*(m_des_pos-m_current_pos) 
                
                # proposed acceleration:
                accel = (vel_des - m_current_vel) / latency
                
                
            if motorid == 2:
                
                m_des_pos = self.pantiltcamera.get_motor_pos(fly_pos)[1]
            
                # controller:
                vel_des = gain*(m_des_pos-m_current_pos) 
                
                # proposed acceleration:
                accel = (vel_des - m_current_vel) / latency
            
            if motorid == 3:
            
                # recover gui offset
                
                try:
                    new_data = self.recvsock.recv(4096)
                    print 'new data recieved: ', new_data
                except:
                    new_data = None
                
                if new_data is not None:
                    self.motor_offset = float(new_data)
            
                foc = self.pantiltcamera.focusmotor.get_focus(fly_pos_est)+self.motor_offset
                #print 'focus: ', foc, 'fly pos est: ', fly_pos_est
                self.m.setpos(foc)
                #vel_des = gain*(foc - m_current_pos)

                # proposed acceleration:
                #accel = (vel_des - m_current_vel) / latency
            
            

            # set motors
            if 1 and motorid != 3:
                m_control = m_current_vel + (vel_des - m_current_vel)*np.exp(-1*np.abs(accel)*damping)
                
                if np.sign(m_control) == self.direction:
                    change_direction = 0
                    
                self.direction = np.sign(m_control)
            
            
                #print motorid, m_control, m_current_vel, vel_des, accel, 'latency: ', latency
                self.m.setvel(m_control, change_direction = change_direction)
                
            
            
            
            self.m_old_vel = m_current_vel
            looptime = time.time()-self.looptime_i
            #print looptime
		    
    def run(self):
     
      
        motorid = self.motorid
        print 'run motor', motorid
        
        while 1:
            new_data = None
            while 1:
                #print os.times()
                try:
                    new_data = self.pos.get_nowait()
                except Queue.Empty:
                    break

            if new_data is not None:
                self.most_recent_data = new_data
                
                # trigger recording of fmf's (but only on one motor):
                if 0:
                    print 'triggered!'
                    socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto('x',('',30041))
                
                # print 'fly pos', new_data
            else:
                pass
                # print 'no new fly position data'
                
                
                
            #case: have new data!
                
                
            if self.most_recent_data is not None:
                self.old_data = self.most_recent_data
                self.followpos(self.most_recent_data)
  
        
            
            
            
    def cleanup(self):
    
        self.m.cleanup()
            
            
            
            
            
            
            
            
