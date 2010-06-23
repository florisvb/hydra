import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg
import scipy.optimize
import mpl_toolkits.mplot3d.axes3d as axes3d
import scipy.interpolate.interpolate as interpolate


class Ray:

    def __init__(self, data_file):
        self.data = np.loadtxt(  data_file,delimiter=',')
        self.points_3d = self.data[:,3:6]
        
        self.fit_line()
        
    def fit_line(self):
        # x = a1*t + b1
        # y = a2*t + b2
        # z = a3*t + b3
        # let t = x -> a1 = 1, b1 = 0
        self.t = np.linspace(0,1,len(self.points_3d[:,0]))
        self.x_fit = np.polyfit(self.t, self.points_3d[:,0], 1)
        self.y_fit = np.polyfit(self.t, self.points_3d[:,1], 1)
        self.z_fit = np.polyfit(self.t, self.points_3d[:,2], 1)
        
        
        
    def get_line(self,v,var):
    
        if var is 'x':
            t = (v-self.x_fit[1]) / self.x_fit[0]
        elif var is 'y':
            t = (v-self.y_fit[1]) / self.y_fit[0]
        elif var is 'z':
            t = (v-self.z_fit[1]) / self.z_fit[0]
        elif var is 't':
            t = v
        
        x = self.x_fit[0]*t + self.x_fit[1]
        y = self.y_fit[0]*t + self.y_fit[1]
        z = self.z_fit[0]*t + self.z_fit[1]
        return x,y,z

## find point closest to intersection ##        

def dist_func(t,ray,p):
    x,y,z = ray.get_line(t,'t')
    dist = np.sqrt((x-p[0])**2 + (y-p[1])**2 + (z-p[2])**2)
    return dist
    
def dist_to_line(ray,p):
    t0 = 0
    tmp = scipy.optimize.fmin( dist_func, t0, args = (ray,p), full_output = 1, disp=0)
    t = tmp[0]
    min_dist = tmp[1]
    return min_dist
    
def dist_all_func(t_arr,rays,p):
    dist = np.zeros(len(rays))
    for i in range(len(rays)):
        dist[i] = dist_to_line(rays[i],p)
    sum_dist = np.sum(dist)
    return sum_dist
    
def dist_to_all_lines(p,rays):
    t0_arr = [0,0,0]
    tmp = scipy.optimize.fmin( dist_all_func, t0_arr, args = (rays,p), full_output = 1, disp=0)
    # tmp = [xopt, fopt, ... ]
    #print 'dist_to_all_lines', tmp[1]
    return tmp[1]
    
def intersection(rays, method = 'slow'):

    if method is 'slow':
        print 'finding intersection: very slow method'
        p0 = [-.2, -1.26, .47]
        tmp = scipy.optimize.fmin( dist_to_all_lines, p0, args = (rays,), disp=0)
        return tmp
    
    if method is 'lstsq':
    
        diag = []
        b = []
        for ray in rays:
            diag.append(ray.x_fit[0]**2 + ray.y_fit[0]**2 + ray.z_fit[0]**2)
            b.append( -1*( ray.x_fit[0]*ray.x_fit[1] + ray.y_fit[0]*ray.y_fit[1] + ray.z_fit[0]*ray.z_fit[1] ))
    
        A = np.diag(diag)
        b = np.array(b)
        
        tmp = scipy.linalg.lstsq(A,b)
        opt_params = tmp[0] 
        print opt_params
        
        points = np.zeros([len(rays), 3])
        for i in range(len(rays)):
            points[i,:] = rays[i].get_line(opt_params[i],'t')
            
            
    

    
## end find intersection point ##
    
def plot_rays(rays, fig = None, ax = None):


    if fig is None:
        fig = plt.figure(0)
    if ax is None:
        ax = axes3d.Axes3D(fig)
    
    try:
        tmp = rays[0]
    except:
        rays = [rays]
        
        
        
        
    for ray in rays:
        # raw data
        ax.scatter3D(ray.points_3d[:,0],ray.points_3d[:,1],ray.points_3d[:,2])

        # fitted line
        x = np.linspace(-1,1,10)
        x,y,z = ray.get_line(x,'x')
        
        ax.plot3D(x,y,z)
        
## focus motor class ##    

class FocusMotor:

    def __init__(self,center,interpolation = 'xinv'):
    
        self.center = np.array(center)
        if self.center.shape[0] > self.center.shape[1]:
            self.center = self.center.T
        self.interpolation = interpolation
        
        
    def dist_to_fly(self,fly_pos):
        distc = scipy.linalg.norm( fly_pos-self.center )
        return distc
        
    def calc_distc(self):
        # calculate distances
        print 'calculating distances'
        self.distc = np.zeros(self.data.shape[0])
        for i in range(self.data.shape[0]):
            self.distc[i] = self.dist_to_fly(self.data[i,3:6])
            
    def get_focus(self,fly_pos):
        
        return self.get_focus_distc(self.dist_to_fly(np.array(fly_pos)))
            

    def get_focus_distc(self,distc):
        
        if self.interpolation is 'xinv':
            return self.coeffs[0] / (distc+self.coeffs[1]) + self.coeffs[2]
        
        if self.interpolation is 'polyfit2':
            return self.coeffs[0]*distc**2 + self.coeffs[1]*distc + self.coeffs[2]
        
        if self.interpolation is 'interp1d':
            try:
                return self.interp(distc)
            except:
                print 'value out of bounds!!!'
                return 0

    def calibrate(self,data_files, plot=1, fig = None):
        print 'calibrating'
        # data structure: [m1,m2,focus,x,y,z]
        
        # if not a list, make it a list
        if type(data_files) is not list:
            print 'not a list, making it a list'
            data_files = [data_files]
    
        # load data
        self.data = None
        for data_file in data_files:
            print data_file
            if self.data is None:
                self.data = np.loadtxt(  data_file, delimiter=',' )
            else:
                tmp = np.loadtxt(  data_file,delimiter=',')
                self.data = np.vstack((self.data,tmp))
        
        self.calc_distc()
        focus = self.data[:,2]
        
        # fit a/(x) + c
        print 'fitting using: ', self.interpolation
        
        if self.interpolation is 'xinv':
            self.coeffs = np.polyfit(self.distc**(-1), focus, 1)
            ## now try fmin fit using coeffs as seed ##
            seed = np.zeros(3)
            seed[0] = self.coeffs[0]
            seed[2] = self.coeffs[1]
            
            tmp = scipy.optimize.fmin( self.fmin_func, seed, full_output = 1, disp=0)
            self.coeffs = tmp[0]
        
        if self.interpolation is 'polyfit2':
            self.coeffs = np.polyfit(self.distc, focus, 2)
            print 'linear coeffs: ', self.coeffs
        
        if self.interpolation is 'interp1d':
            print 'interpolating using interp1d'
            self.interp = interpolate.interp1d(self.distc, focus, kind = 'linear', fill_value = 0)
     
            
        if plot == 1:
            if fig is None:    
                fig = plt.figure(1)
            plt.scatter(self.distc,focus)
            xi = np.linspace(min(self.distc),max(self.distc),50)
            yi = [self.get_focus_distc(x) for x in xi]
            
            plt.title('Calibration data for Pan Tilt Focus')
            plt.xlabel('distance to camera center, m')
            plt.ylabel('focus motor setting, radians')
            plt.plot(xi,yi)
            fig.show()
                
        
        
        return 1
        
    def fmin_func(self, coeffs):
        
        focus = coeffs[0] / (self.distc+coeffs[1]) + coeffs[2]
        err = focus - self.data[:,2]
        abs_err = np.abs(err)
        err_sum = np.sum(abs_err)
        
        return err_sum



def main (plot=1, data_files = None):

    if data_files is None:
        print 'using default data files'
        data_files = ['/home/floris/calibrations/hydra_cal_files/20100506_multiaxis/motor6d_info_20100506_183030_axis2.txt',
                        '/home/floris/calibrations/hydra_cal_files/20100506_multiaxis/motor6d_info_20100506_183030_axis1.txt',
                        '/home/floris/calibrations/hydra_cal_files/20100506_multiaxis/motor6d_info_20100506_183030_axis3.txt']


    rays = []    
    for data_file in data_files[0:3]:                      
        ray = Ray(data_file)
        rays.append( ray )
    print 'number of rays: ', len(rays)
        
    p = intersection(rays)
    print 'intersection: ', p
        
    
    if plot:
        fig = plt.figure(0)
        ax = axes3d.Axes3D(fig)
        plot_rays(rays, fig=fig, ax = ax)
        
        # axes 3d will not allow plotting a single point - so append the origin as hack  
        hack = np.zeros([2,3])
        hack[0,:] = p
        ax.scatter3D(hack[:,0], hack[:,1], hack[:,2])
        
        ax.set_xlabel('flydra x coordinates, meters')
        ax.set_ylabel('flydra y coordinates, meters')
        ax.set_zlabel('flydra z coordinates, meters')
        
        
        fig.show()
        
    focusmotor = FocusMotor(p)
    focusmotor.calibrate(data_files)
            
        
    # find distance from fly to intersection point
    
        
    return focusmotor
        
        
        
        
        
        
        
if __name__=='__main__':
    main()
