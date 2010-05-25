import volume_post_trigger as post_trigger
import numpy as np

center = np.array([0,0,0])
sides = np.array([0.1, 0.1, 0.1])

speed = 0.01

post_center = np.array([0,0,0])
post_diam = 0.0
post_buff = 0.0


# 5 cm/s; 200 ms: we have a flying fly

trigger = post_trigger.Trigger(30041)
trigger.set_voi(center,sides,post_center,post_diam,post_buff,speed)
msg = 'x'
trigger.run(msg)  
