import numpy
import scipy.linalg
import sys

# Extract (linear) camera parameters.

####################################################################

# sample data from "Multiple View Geometry in Computer Vision" Hartley
# and Zisserman, example 6.2, p. 163

def determinant( A ):
    orig_determinant = numpy.linalg.det
    return orig_determinant( numpy.asarray( A ) )

def decomp(P,SHOW=0):

    if len( P ) < 2:
        P = numpy.array( [[ 3.53553e2,   3.39645e2,  2.77744e2,  -1.44946e6 ],
                          [-1.03528e2,   2.33212e1,  4.59607e2,  -6.32525e5 ],
                          [ 7.07107e-1, -3.53553e-1, 6.12372e-1, -9.18559e2 ]] )


    # camera center
    X = determinant( [ P[:,1], P[:,2], P[:,3] ] )
    Y = -determinant( [ P[:,0], P[:,2], P[:,3] ] )
    Z = determinant( [ P[:,0], P[:,1], P[:,3] ] )
    T = -determinant( [ P[:,0], P[:,1], P[:,2] ] )

    C_ = numpy.transpose(numpy.array( [[ X/T, Y/T, Z/T ]] ))

    M = P[:,:3]

    # do the work:
    # RQ decomposition: K is upper-triangular matrix and R is
    # orthogonal. Both are components of M such that KR=M

    K,R = scipy.linalg.rq(M) # added to scipy 0.5.3
    Knorm = K/K[2,2]

    # So now R is the rotation matrix (which is orthogonal) describing the
    # camera orientation. K is the intrinsic parameter matrix.

    t = numpy.dot( -R, C_ )

    # reconstruct P via eqn 6.8 (p. 156)
    P_ = numpy.dot( K, numpy.concatenate( (R, t), axis=1 ) )

    return K,R,t

    if SHOW:
        print 'P (original):'
        print P
        print

        print 'C~ (center):'
        print C_
        print

        print 'K (calibration):'
        print K
        print

        print 'normalized K (calibration):'
        print Knorm
        print

        print 'R (orientation):' # same as rotation matrix
        print R
        print

        print 't (translation in world coordinates):'
        print t
        print

        print 'P (reconstructed):'
        print P_
        print
        

def pmat2cam_center(P):
    # return camera center in world coordinates (from flydra reconstruct)
    """

    See Hartley & Zisserman (2003) p. 163
    """
    assert P.shape == (3,4)
    determinant = numpy.linalg.det

    # camera center
    X = determinant( [ P[:,1], P[:,2], P[:,3] ] )
    Y = -determinant( [ P[:,0], P[:,2], P[:,3] ] )
    Z = determinant( [ P[:,0], P[:,1], P[:,3] ] )
    T = -determinant( [ P[:,0], P[:,1], P[:,2] ] )

    C_ = nx.transpose(nx.array( [[ X/T, Y/T, Z/T ]] ))
    return C_
        
