from enthought.traits.api import *
from enthought.traits.ui.api import *
import socket

class Camera(HasTraits):
    """ Camera object """

    m_low = -0.4
    m_high = 0.4
        
    motor_offset = Range( m_low, m_high, 0, mode='slider', set_enter=True)
    
    traits_view = View(Group(Item('motor_offset',
                                  editor = RangeEditor( low_name    = 'm_low',
                                                        high_name   = 'm_high',
                                                        mode        = 'slider' ),
                                  ),
                             ))
    
    def __init__(self):
        super(Camera,self).__init__()
        self.host = ''
        self.port = 30045
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        print type(self.sender)
    
    def _motor_offset_changed(self):
        print self.motor_offset
        self.sender.sendto(str(self.motor_offset),(self.host,self.port))
        
    


if __name__ == "__main__":
    camera = Camera()
    camera.configure_traits()

    
