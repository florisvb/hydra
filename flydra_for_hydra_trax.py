

import socket, Queue
import flydra.kalman.data_packets as data_packets
import flydra.common_variables
import Pyro.core
import numpy as np
import time

pi = np.pi
dummy_default_position = 1,0,0
freq = 0



class Listener(object):
    def __init__(self,sockobj,dummy=False):
        self.s = sockobj
        self.q = Queue.Queue()
               
        self.preferred_objid = None
        
        self.fly_pos = None
        self.voi_center = np.array([0,0,0])
        self.voi_sides = np.array([10**2,10**2,10**2])
        
        self.voi_mask_center = np.array([0,0,0])
        self.voi_mask_sides = np.array([0,0,0])
        self.voi_post_center = np.array([0,0,0])
        self.voi_post_diam = 0
        self.voi_post_buff = 0
        self.voi_speed = 0
        
        self.dummy = dummy
        print 'Listener created'
        
        
        # open file to save landing obj_ids
        fname = time.strftime( 'landing_objid_%Y%m%d_%H%M%S.txt' )
        self.fd = open( fname, mode='w' )
        print 'saving pref object ids to: ',fname
        
        
    def set_voi(self, center = [0, 0, 0], sides = [0,0,0], post_center = [0,0,0], post_diam = 0, post_buff = 0, speed = 0):
        self.voi_center = center
        self.voi_post_center = post_center
        self.voi_post_diam = post_diam
        self.voi_post_buff = post_buff
        self.voi_sides = sides
        self.voi_speed = speed
        
        
        
        
    def choose_pref_objid(self, obj_ids, state_vecs, meanPs):
    
        meanPs = np.array(meanPs)
    
        self.preferred_objid = None
        
        if meanPs is not None: 
            meanPs_index_sorted = meanPs.argsort()
            
            for ind in meanPs_index_sorted:
                
                
                x,y,z,xvel,yvel,zvel= state_vecs[ind]
                pos = np.array([x,y,z])
                dist = pos - self.voi_center
                err = self.voi_sides - np.abs(dist)
                voitest = np.sum(np.sign(err))  
                
                speed = np.sqrt(xvel**2 + yvel**2 + zvel**2)
                
                # check to see if in/on cylinder:
                mask = 1 # one means not in on/post
                if z <= self.voi_post_center[2]+self.voi_post_buff:
                    
                    if np.sqrt( (x-self.voi_post_center[0])**2+ (y-self.voi_post_center[1])**2) <= np.sqrt(self.voi_post_diam/2.0)+self.voi_post_buff:
                        mask = 0 # in/on post
                
                
                if voitest == 3 and speed>=self.voi_speed and mask==1:
                    
                    self.preferred_objid = obj_ids[ind]
                    buf = '%i,%s'%(self.preferred_objid, time.strftime('%Y%m%d_%H%M%S'))
                    self.fd.write(buf + '\n')
                    self.fd.flush()
                    print 'new obj id: ',buf, 'speed: ', speed
                    
                    break


    def run(self):
        while 1:
            #print 'running'
            # get data when available, put into buffer
            # process data
            # stick data into the gui

            #print 'listening for packet on',self.s       
                        
            if not self.dummy:
                buf, addr = self.s.recvfrom(4096)
            else:
                time.sleep(0.05) # pause 50 msec
            #print 'got packet:',buf
            #self.q.put( buf )
            

            
            
                
            if self.fly_pos is not None:
            
                if self.dummy:
                    theta = time.time() % (2*pi)
                    x = 0.3*np.cos( theta )
                    y = 0.05*np.sin( theta )
                    z = 0.05*np.sin( theta/2.3 )    
                    voitest = 3    
            
                else: # actually use flydra
                    #print('using flydra!')
                    packets = data_packets.decode_super_packet( buf )
                    for packet in packets:
                        tmp = data_packets.decode_data_packet(packet)
                        # obj_ids: order from one that's been around the longest first
                        # state_vecs = [x,y,z,xvel,yvel,zvel]
                        # meanPs = error for each object
                        corrected_framenumber, timestamp_i, timestamp_f, obj_ids, state_vecs, meanPs = tmp
                        
                    if self.preferred_objid is None:
                        self.choose_pref_objid(obj_ids, state_vecs, meanPs)
                        
                    # get the preferred object id, use that
                                    
                    try:
                        pref_objid_index = obj_ids.index(self.preferred_objid)
                    except ValueError:
                        #print "That fly doesn't exist anymore... finding a new fly"
                        self.choose_pref_objid(obj_ids, state_vecs, meanPs)
                        #print("Found a new fly with object ID: ", self.preferred_objid)
                        #print("Fly list: ", obj_ids)
                        
                    if self.preferred_objid is None:
                        continue
                    
                    pref_objid_index = obj_ids.index(self.preferred_objid)
                        
        
                    latency = timestamp_f - timestamp_i
                    #if latency >= 0.1:
                    #    raise ValueError, 'Flydra latency > 100ms, something is probably wrong!'
                    x,y,z,xvel,yvel,zvel= state_vecs[pref_objid_index]
                    #print(x,y,z)
                    

                    self.fly_pos.pos.put([x,y,z,xvel,yvel,zvel,latency])
                    #print 'flydras fly_pos', self.fly_pos.pos

                
    def get_list_of_bufs(self):
        result = []
        while 1:
            try:
                result.append( self.q.get_nowait() )
            except Queue.Empty:
                break
        return result
    def get_most_recent_single_fly_data(self):
        superpacket_bufs = self.get_list_of_bufs()
        if len(superpacket_bufs)==0:
            # no new data
            return
        buf = superpacket_bufs[-1] # most recent superpacket
        packets = data_packets.decode_super_packet( buf )
        packet = packets[-1] # most recent packet
        tmp = data_packets.decode_data_packet(packet)
        #(corrected_framenumber, timestamp, obj_ids, state_vecs, meanPs) = tmp
        return tmp

    def push_fly_pos(self,fly_trax_pos):
        self.fly_pos = fly_trax_pos
        print 'INTIAL fly_pos', self.fly_pos

    def get_fly_pos(self):

        return self.fly_pos
        
        

class DummyMainbrain:
    def log_message(self,*args,**kwds):
        return
        
        

