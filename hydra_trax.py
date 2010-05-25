# need to active virtual environment!
# ~/PY/bin/activate


import flydra_for_hydra_trax as ffh
import flydra.common_variables
import flydra.kalman.data_packets as data_packets
import Pyro.core
import numpy as np
import time
import threading
import socket
import flydra
import Pyro
import hydra_cal
import hydra_motors_stepper as h
    
# set up flydra
connect_to_mainbrain = 1


# initialize fly stats
fly_trax_pos = [0,0,0]

if connect_to_mainbrain:
	# make connection to flydra mainbrain
	my_host = '' # get fully qualified hostname
	my_port = 8322 # arbitrary number

	# create UDP socket object, grab the port
	sockobj = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	print 'binding',( my_host, my_port)
	sockobj.bind(( my_host, my_port))

	# connect to mainbrain
	mainbrain_hostname = 'sericomyia'
	mainbrain_port = flydra.common_variables.mainbrain_port
	mainbrain_name = 'main_brain'
	remote_URI = "PYROLOC://%s:%d/%s" % (mainbrain_hostname,
		                         mainbrain_port,
		                         mainbrain_name)
	Pyro.core.initClient(banner=0)
	mainbrain = Pyro.core.getProxyForURI(remote_URI)
	mainbrain._setOneway(['log_message'])
	my_host_fqdn = socket.getfqdn(my_host)
	mainbrain.register_downstream_kalman_host(my_host_fqdn,my_port)
	
	print 'creating listener with sockobj',sockobj
	listener = ffh.Listener(sockobj, dummy=False)
	print 'created listener'

else:
	listener = ffh.Listener(None,dummy=True)
	mainbrain = ffh.DummyMainbrain()
listen_thread = threading.Thread(target=listener.run)
listen_thread.setDaemon(True)
listen_thread.start()
listener.push_fly_pos(fly_trax_pos)
		
		
		
# initialize motors and calibration		
		
		
Mhat = hydra_cal.getMhat()

print 'Mhat'
print Mhat
print

T = hydra_cal.getw2c(Mhat)
c = hydra_cal.getFocus()

T = np.matrix(T)
Mhat = np.matrix(Mhat)


h1 = h.HydraMotor(1)
#h1.findzeros()

h2 = h.HydraMotor(2)
#h2.findzeros()

h3 = h.HydraMotor(3)



while 0:
    
    print 'fly pos', fly_trax_pos
    
    time.sleep(.1)

# track!

while 1:


    fly_pos = list(listener.get_fly_pos())
    fly_pos.append(1)
    fly_pos = np.array(fly_pos)
    
    
    print 

    print 'fly pos', fly_pos
    
    
    
    
    
    if 0:
        fly_pos = np.matrix(fly_pos).T
        q = Mhat*fly_pos
        r = q[0]
        s = q[1]
        t = q[2]
        print 'q', q
        u = r/t
        v = s/t
        alpha = np.arctan2(u,1)
        beta = np.arctan2(v,1)
        
        print alpha
        print beta
    
        h1.setpos(alpha)
        h2.setpos(beta)
    
    
    
        # focus
        
        Xc = T*fly_pos
        print Xc[2]
        print c[0]
        print c[1]
        
        foc = c[0]*Xc[2] + c[1]
        
        h3.setpos(foc)


        print

    time.sleep(0.01)












