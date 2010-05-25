import Pyro.core

import numpy as np
import flydra_for_hydra_trax as ffh
import threading
import socket
import flydra
import time
import Queue

import copy
import scipy.linalg as linalg
import sys
pi = np.pi







class Trigger:

    

    def __init__(self, port):
        
        self.pos = Queue.Queue()

        
        print 'initializing flydra'
        self.initialize_flydra()
        print 'done initializing flydra'
        
        self.old_data = [0,0,0,0,0,0]
        self.most_recent_data = None
        
        # initialize post trigger:
        self.trigger_host = ''
        self.trigger_port = port # use for post trigger 30041
        self.trigger_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                
        

    def initialize_flydra(self):
        
            # set up flydra
		    connect_to_mainbrain = 1

		    if connect_to_mainbrain:
			    # make connection to flydra mainbrain
			    my_host = '' # get fully qualified hostname
			    my_port = 8320 # arbitrary number

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
		    
    def set_voi(self, center, sides, post_center, post_diam, post_buff, speed):
        self.listener.set_voi(center, sides, post_center, post_diam, post_buff, speed)
	    
		    
		    

        
		    
    def run(self, msg):


        while 1:
            new_data = None
            while 1:
                try:
                    new_data = self.pos.get_nowait()
                except Queue.Empty: 
                    break

            #case: have new data!
            if new_data is not None:
            
                
                print 'triggered!'
                self.trigger_sender.sendto(msg,(self.trigger_host,self.trigger_port))

        
            
            
            
    def cleanup(self):
    
        self.m.cleanup()
            
            
            
            
            
            
            
            
