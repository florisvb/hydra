import socket, Queue
import flydra.kalman.data_packets as data_packets
import flydra.common_variables
import Pyro.core
import numpy as np
import time
import simple_step
import pdb
import atexit
import sys
import cPickle as pickle


# talk to bai controller
import BAI		    


# KP = 600,000; KI = 10,000; KPOS = 4000

bk = pdb.set_trace

BAUDRATE = 38400
pi = np.pi
dummy_default_position = 1,0,0

# motor constants
gr = 7.2
ind_per_rad = 4000.0/(2.0*pi)
rad_per_ind = 1.0/ind_per_rad
KP = 5
KI = 0
KPV = 0
freq = 2



class Listener(object):
    def __init__(self,sockobj):
        self.s = sockobj
        self.q = Queue.Queue()
    def run(self):
        while 1:
            #print 'listening for packet on',self.s
            buf, addr = self.s.recvfrom(4096)
            #print 'got packet:',buf
            self.q.put( buf )
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

class InterpolatingListener(object):
    def __init__(self,sockobj,dummy=False):
        if sockobj is not None:
            self.listener = Listener(sockobj)
        self.last_data_time = -np.inf
        self.current_state_vec = None
        self.dummy=dummy
        self.dummy_last_xyz = None
        self.dummy_last_time = None
    def run(self):
        self.listener.run()
    def get_fly_xyz(self,prefer_obj_id=None):
        """
        Returns
        =======
        obj_id
        fly_xyz
        framenumber
        """

        if self.dummy:
            t = time.time()
            timestamp = t
            alpha = (2*pi*t)*freq
            x = dummy_default_position[0] + 0*np.cos( alpha )
            y = dummy_default_position[1] + 1*np.sin( alpha )
            z = dummy_default_position[2] + 0.05#*np.sin( alpha/2.3 )
            if self.dummy_last_xyz is not None:
                dt = timestamp-self.dummy_last_time
                lastx,lasty,lastz = self.dummy_last_xyz
                dx=x-lastx
                dy=y-lasty
                dz=z-lastz
                if dt==0:
                    vx,vy,vz=0,0,0
                else:
                    vx=dx/dt
                    vy=dy/dt
                    vz=dz/dt
            else:
                vx,vy,vz = 0,0,0
            self.dummy_last_xyz = (x,y,z)
            self.dummy_last_time = timestamp
            state_vecs = [ (x,y,z) ]
            corrected_framenumber = 0
            meanP = 0
            return 0, (x,y,z), 0, (vx,vy,vz)

        new_data_all = self.listener.get_most_recent_single_fly_data()
        now = time.time()
        if new_data_all is not None:
            (corrected_framenumber, timestamp,
             obj_ids, state_vecs, meanPs) = new_data_all
            #corrected_framenumber, timestamp, state_vecs, meanP = new_data_all
            self.last_data_time = now
            if prefer_obj_id is not None:
                try:
                    idx = obj_ids.index(prefer_obj_id)
                except ValueError:
                    idx = 0
            else:
                idx = 0
            self.current_framenumber = corrected_framenumber
            self.current_obj_id = obj_ids[idx]
            self.current_state_vec = np.array(state_vecs[idx]) #convert to numpy
        dt = now-self.last_data_time
        if dt > 0.1:
            # return None if no recent target (recent defined as 100 msec)
            return None

        if dt <= 0.0:
            # return pure X,Y,Z
            return (self.current_obj_id,
                    self.current_state_vec[:3],
                    self.current_framenumber,
                    self.current_state_vec[3:])


        state_vec = self.current_state_vec
        dx = dt*state_vec[3:] # velocity*time
        newx = state_vec[:3]+dx
        return self.current_obj_id, newx, self.current_framenumber, state_vec[3:]

class DummyMainbrain:
    def log_message(self,*args,**kwds):
        return

class ControlLoop:

    def __init__(self):

        self.bai0 = BAI.BAI(baudrate=BAUDRATE,port='/dev/ttyUSB0')
	self.bai1 = BAI.BAI(baudrate=BAUDRATE,port='/dev/ttyUSB1')

        self.devA = simple_step.Simple_Step(serial_number='0.0.A')
        self.devA.set_zero_pos(self.devA.get_pos())
        self.devA.set_mode(simple_step.VELOCITY_MODE)
        self.devA.set_vel_setpt(0,io_update=True)
        self.devA.start()

	self.devB = simple_step.Simple_Step(serial_number='0.0.B')
        self.devB.set_zero_pos(self.devB.get_pos())
        self.devB.set_mode(simple_step.VELOCITY_MODE)
        self.devB.set_vel_setpt(0,io_update=True)
        self.devB.start()

        # for saving data
        
        # save vals
        self.t_array = []
        self.pos_cmd_array = []
        self.pos_tru_array = []
        self.vel_cmd_array = []
        self.vel_tru_array = []
        self.motor_pos_err_array = []
           




        atexit.register(self.atexit_func)

    def atexit_func(self):
        
        t_nparray = np.array(self.t_array)
        pos_cmd_nparray = np.array(self.pos_cmd_array)
        pos_tru_nparray = np.array(self.pos_tru_array)
        vel_cmd_nparray = np.array(self.vel_cmd_array)
        vel_tru_nparray = np.array(self.vel_tru_array)
        motor_pos_err_nparray = np.array(self.motor_pos_err_array)
        

        data = {
                't'        : t_nparray,
                'pos_cmd'  : pos_cmd_nparray,
                'pos_tru'  : pos_tru_nparray,
                'vel_cmd'  : vel_cmd_nparray,
                'vel_tru'  : vel_tru_nparray,
                'motor_pos_err' : motor_pos_err_nparray,
                }

        datafile = open('tmp.pkl','wb')
        pickle.dump(data, datafile)
        datafile.close()
        print 'data saved'

        # Close device
        self.dev.close()

        del self.dev
        dev2 = simple_step.Simple_Step()
        dev2.stop()

        self.bai0.close()
	self.bai1.close()
        # dump data

        print('goodbye')
        

    def doit(self,connect_to_mainbrain=True):

        if connect_to_mainbrain:
            # make connection to flydra mainbrain
            my_host = '' # get fully qualified hostname
            my_port = 8322 # arbitrary number

            # create UDP socket object, grab the port
            sockobj = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print 'binding',( my_host, my_port)
            sockobj.bind(( my_host, my_port))

            # connect to mainbrain
            mainbrain_hostname = 'brain1'
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

            listener = InterpolatingListener(sockobj)
            listen_thread = threading.Thread(target=listener.run)
            listen_thread.setDaemon(True)
            listen_thread.start()
        else:
            listener = InterpolatingListener(None,dummy=True)
            mainbrain = DummyMainbrain()



        # Open device 

        # Set up device interfaces 


        
        

        
        start_check = 0
        motor_rot_count = 0
        atmel_rot_count = 0
        integral_err = 0
        looptime = 0.008



        time_start = time.time()

        while 1:
            looptimestart = time.time()
            tmp = listener.get_fly_xyz()
            if tmp is not None:
                obj_id,fly_xyz,framenumber,fly_vel_vec = tmp
                fly_check = 1
            else:
                fly_check = None
                fly_xyz = fly_xyz_prev
                fly_vel_vec = fly_vel_vec_prev
                obj_id = None
                framenumber = None
            del tmp

            if fly_check is None:
                print 'No update'

            fly_vel_vec_prev = fly_vel_vec
            fly_xyz_prev = fly_xyz

            # account for location of flydra, and location of hydra: fly is coords of fly w.r.t. hydra
            hydra_xyz = np.array([0,0,0])
            fly = fly_xyz + hydra_xyz
            #print(fly_xyz)
            #print(fly)

            # unpack fly pos/vel
            x,y,z = fly
            vx,vy,vz = fly_vel_vec

            # calculate radius, theta, phi
            rad = (x**2+y**2)**(0.5)
            theta = np.arctan2(y,x)
            
            phi = np.arctan2(z,rad)
            if phi<0:
                phi = 2.0*pi-np.abs(phi)


            # get ZERO pos for motor, atmel, and fly
            if start_check<1:
                print('check point passed')
                motor_pos_zero = -1*self.bai0.get_position()*rad_per_ind/gr
                theta_zero = 0
                atmel_pos_zero = self.dev.get_pos()*rad_per_ind/gr
            start_check += 1
            theta = theta-theta_zero
            
            # calculate normalized radius vector (u1), and the vector perpendicular to that
            u1 = np.array([x/rad,y/rad])
            u1_perp = np.array([-1*u1[1],u1[0]])

            v_xy = np.array([vx,vy])

            # get the proper rotation for the fly - crossproduct of fly position and velocity (from hydra's origin)
            dirpos = np.array([x,y,0])
            dirvel = np.array([vx,vy,0])
            dirrot = np.cross(dirpos,dirvel)
            
            # get the portion of velocity perpendicular to the radius from hydra to fly, find theta dot
            v_xy_rot = np.abs(np.dot(u1_perp,v_xy))*np.sign(dirrot[2])
            thetadot = v_xy_rot/rad


        

            
            if 0:
                theta = 0
                thetadot = 0

                if start_check>100:
                    thetadot = 5
                    theta_full = thetadot*(time.time()-time_start)
                    theta = np.fmod(theta_full, 2.0*pi)
             



            ## TALK TO MOTOR

            
            if 0:
                # ask motor where is, compare to zero, clear rotations
                motor_pos = -1*self.bai0.get_position()*rad_per_ind/gr-motor_pos_zero    

                motor_pos_noloop = motor_pos - 2.0*pi*motor_rot_count
                if np.abs(motor_pos_noloop) > 2.0*pi:
                    print('MOTOR COUNT: ', motor_rot_count)
                    motor_rot_count += np.sign(motor_pos_noloop)
                    motor_pos_noloop = motor_pos - 2.0*pi*motor_rot_count
                # error between motor and theta
                motor_pos_err = motor_pos_noloop-(theta-theta_zero)

            if 1:
                # ask atmel where is, compare to zero, clear rotations
                atmel_pos = self.dev.get_pos()*rad_per_ind/gr-atmel_pos_zero 
           
                atmel_pos_noloop = atmel_pos - 2.0*pi*atmel_rot_count
                if np.abs(atmel_pos_noloop) > 2.0*pi:
                    print('atmel COUNT: ', atmel_rot_count)
                    atmel_rot_count += np.sign(atmel_pos_noloop)
                    atmel_pos_noloop = atmel_pos - 2.0*pi*atmel_rot_count
                # error between atmel and theta
                atmel_pos_err = atmel_pos_noloop-(theta-theta_zero)

                motor_pos_noloop = atmel_pos_noloop
                motor_pos_err = atmel_pos_err

            if np.abs(motor_pos_err) > 2.0*pi:
                motor_pos_err += np.sign(motor_pos_err)*2.0*pi

            if np.abs(motor_pos_err) > pi:
                motor_pos_err = -1*np.sign(motor_pos_err)*(2.0*pi-np.abs(motor_pos_err))        

            ## END MOTOR TALK



            ## CONTROL LAWS

            # Integral control, with wind up control
            integral_err += motor_pos_err
            #if np.abs(integral_err) >= pi:
             #   integral_err = np.sign(integral_err)*pi
            KI_ctrl = (-1)*integral_err*KI

            # Proportional control on position          
            KP_ctrl = (-1)*motor_pos_err*KP

            
            
            
            ## END CONTROL LAWS        

            # Feedback Control Law - vel_ctrl is in radians/sec
            control_effort = KP_ctrl + KI_ctrl
            vel_ctrl_rads = thetadot + control_effort
            vel_ctrl = gr*ind_per_rad*(vel_ctrl_rads)

            # get direction
            if vel_ctrl > 0:
                dir_setpt = simple_step.POSITIVE
            else:
                dir_setpt = simple_step.NEGATIVE

            # Set limits on controlled velocity 
            vel_ctrl_max = 30000.0
            vel_ctrl_min = 16
            if np.abs(vel_ctrl) > vel_ctrl_max:
                vel_ctrl = vel_ctrl_max*np.sign(vel_ctrl)
            elif np.abs(vel_ctrl) < vel_ctrl_min:
                vel_ctrl = 0

            
            # send commanded velocity to motor            
            self.dev.set_dir_setpt(dir_setpt,io_update=False)
            self.dev.set_vel_setpt(np.abs(vel_ctrl),io_update=True)




            # save vals
            self.t_array.append(time.time()-time_start)
            self.pos_cmd_array.append(theta)
            self.pos_tru_array.append(motor_pos_noloop)
            self.vel_cmd_array.append(thetadot)

            motor_dir = self.dev.get_dir()
            if motor_dir is 'positive':
                motor_dir_val = 1.0
            else:
                motor_dir_val = -1.0    
            
            motor_vel = self.dev.get_vel()/(gr*ind_per_rad)*motor_dir_val

            self.vel_tru_array.append(motor_vel)
            self.motor_pos_err_array.append(motor_pos_err)
           

            print('motor vel', motor_vel,  'theta', theta, 'Thetadot: ',thetadot, 'KP ctrl: ', KP_ctrl, 'KI ctrl: ', KI_ctrl)



def main():
    from optparse import OptionParser
    usage = '%prog [options]'

    parser = OptionParser(usage)

    parser.add_option("--standalone", action='store_true',
                      help="do not attempt to connect to mainbrain",
                      default=False)
    (options, args) = parser.parse_args()

    connect_to_mainbrain = not options.standalone
    controlloop = ControlLoop()
    controlloop.doit(connect_to_mainbrain=connect_to_mainbrain)

if __name__=='__main__':
    main()
