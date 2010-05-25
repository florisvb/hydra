import traxclass
import processing
import sys
import time
import signal
import numpy as np




class TraxProcess(processing.Process):
    def set_motorid(self,motorid):
        self.motorid=motorid
    def run(self):
        f = traxclass.Trax(self.motorid)
        try:
            f.initmotor()
            center = np.array([0,0,0])
            sides = np.array([10, 10, 10])
            speed = 0.01
            f.set_voi(center = center, sides = sides, speed = speed)
            f.run()
        finally:
            f.cleanup()
            
p1 = TraxProcess()
p1.set_motorid(1)
p1.setDaemon(True)
p1.start()

p2 = TraxProcess()
p2.set_motorid(2)
p2.setDaemon(True)
p2.start()

p3 = TraxProcess()
p3.set_motorid(3)
p3.setDaemon(True)
p3.start()




def signal_handler(signal, frame):
        ans = raw_input('Quit called')
        sys.exit(0)
            
signal.signal(signal.SIGINT, signal_handler)

p1.join()




