# Jose Joseph Thandapral
# CS5335
# HW4: Point Cloud Analysis

from typing import Tuple
import numpy as np
import random
import utils
from scipy.spatial import cKDTree


def q1_a(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Fit a least squares plane by taking the Eigen values and vectors
    of the sample covariance matrix

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space

    Returns
    -------
    normal : np.ndarray
        array of shape (3,) denoting surface normal of the fitting plane
    center : np.ndarray
        array of shape (3,) denoting center of the points
    '''
    #Calculating the mean vector of the X,Y,Z dimensions of points and 
    #centering it at the origin of co-ordinates
    center = np.mean(P,axis=0)
    #initialize normal to be the same a center
    normal = center
    #Calculate the covariance matrix 
    covm = np.cov(P.T)
    #calculate the singular value decompostion of the covariance matrix
    u, s, v = np.linalg.svd(covm, full_matrices=False)
    # the third column of `vh` contains the principal component corresponding to the smallest singular value, which is the surface normal
    normal = v[2]
    print("normal vector of plane of points= ", normal)
    print("center of the plane of point cloud = ", center)
    return [normal,center]

def plane_equation(P1, P2, P3):  

    #ax+by+cz = d
    x1, y1, z1 = P1
    x2, y2, z2 = P2
    x3, y3, z3 = P3
    a1 = x2 - x1 
    b1 = y2 - y1 
    c1 = z2 - z1 
    a2 = x3 - x1 
    b2 = y3 - y1 
    c2 = z3 - z1 
    a = b1 * c2 - b2 * c1 
    b = a2 * c1 - a1 * c2 
    c = a1 * b2 - b1 * a2 
    d = (- a * x1 - b * y1 - c * z1) 
    return a, b, c, d




def q1_c(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Fit a plane using RANSAC

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space

    Returns
    -------
    normal : np.ndarray
        array of shape (3,) denoting surface normal of the fitting plane
    center : np.ndarray
        array of shape (3,) denoting center of the points
    '''
    #Calculating the mean vector of the X,Y,Z dimensions of points and 
    #centering it at the origin of co-ordinates
    center = np.mean(P,axis=0)
    #Set the maximum number of RANSAC iterations
    numiter = 10000
    #Set the tuning parameter for inliers
    alpha = 0.01
    maxinliers = 0
    print("Performing RANSAC over given point cloud")
    #Iterate numiter # of times
    for i in range(numiter):
        idx = random.sample(range(0, P.shape[0]), 3)
        a,b,c,d = plane_equation(P[idx[0]],P[idx[1]],P[idx[2]])
        inliers = 0
        for j in range(P.shape[0]):
            p = P[j]
            distance = abs((a*p[0]+b*p[1]+c*p[2]+d)/np.sqrt(a*a+b*b+c*c))
            if (distance < alpha):
                inliers += 1
        if inliers >= maxinliers:
            maxinliers = inliers
            normal = np.array([a,b,c])
    print("normal vector of plane of points= ", normal)
    print("center of the plane of point cloud = ", center)
    return [normal,center]

def q2(P: np.ndarray, N: np.ndarray) -> Tuple[np.ndarray, float]:
    '''
    Localize a sphere in the point cloud. Given a point cloud as
    input, this function should locate the position and radius
    of a sphere

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space
    N : np.ndarray
        Nx3 matrix denoting normals of pointcloud

    Returns
    -------
    center : np.ndarray
        array of shape (3,) denoting sphere center
    radius : float
        scalar radius of sphere
    '''

    #Set the maximum number of RANSAC iterations
    numiter = 1000
    #Set the tuning parameter for inliers
    alpha = 0.01
    maxinliers = 0

    print("Performing RANSAC over given point cloud  for Sphere fitting")
    #Getting a linearly spaces array of 10 values as radii to iterate over
    radii = np.linspace(5,11,10)*0.01
    for i in range(numiter):
        if i%100 == 0:
             print("Iteration#", i)
        inlier = 0
        idx = random.sample(range(0, P.shape[0]), 1)[0]
        idr = random.sample(range(0, 9), 1)[0]
        p = P[idx]
        r = radii[idr]
        n = N[idx]
        c = p+r*n
        for j in range(P.shape[0]):
            rp = P[j]
            distance = np.linalg.norm(rp-c)
            if distance >= r-alpha and distance <= r+alpha:
                inlier += 1
        if inlier >= maxinliers:
            maxinliers = inlier
            radius = r
            center = c

    print()
    print("Fitted sphere's center:", center)
    print("Fitted sphere's Radius:", radius)
    print("Maximum number of inliers = ",maxinliers)

    return [center,radius]


def q3(P: np.ndarray, N: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    '''
    Localize a cylinder in the point cloud. Given a point cloud as
    input, this function should locate the position, orientation,
    and radius of the cylinder

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting 100 points in 3D space
    N : np.ndarray
        Nx3 matrix denoting normals of pointcloud

    Returns
    -------
    center : np.ndarray
        array of shape (3,) denoting cylinder center
    axis : np.ndarray
        array of shape (3,) pointing along cylinder axis
    radius : float
        scalar radius of cylinder
    '''
    #Set the maximum number of RANSAC iterations
    numiter = 1000
    #Set the tuning parameter for inliers
    alpha = 0.01
    maxinliers = 0
    print("Performing RANSAC over given point cloud  for Cylinder fitting")
    #Getting a linearly spaces array of 10 values as radii to iterate over
    radii = np.linspace(5,10,10)*0.01

    for i in range(numiter):
        if i%100 == 0:
             print("Processing iteration", i)
        inlier = 0
        idx = random.sample(range(0, P.shape[0]), 2)
        idr = random.sample(range(0, 9), 1)[0]
        p1 = P[idx[0]]
        r = radii[idr]
        n1 = N[idx[0]]  # 1st normal of cylinder
        n2 = N[idx[1]]  # 2nd normal of cylinder
        axis = np.cross(n1,n2)
        c = p1+r*n1  #center point of cylinder
        pm = np.eye(3)-np.outer(axis,axis.T)#projection matrix 
        pc = np.dot(pm,c)  #projection center point 
        PP = (np.dot(pm,P.T)).T  # projected pointcloud 
        for j in range(P.shape[0]):
            rp = PP[j]   # random point selected in projected pointcloud 
            #calculating the distance as an L2 norm
            distance = np.linalg.norm(rp-pc)

            if distance >= r-alpha and distance <= r+alpha:
                inlier += 1
        
        if inlier >= maxinliers:  # selecting best inlier parameters 
            maxinliers = inlier
            radius = r
            center = pc
            caxis = axis 

    print()
    print("Fitted cylinder's center:", center)
    print("Fitted cylinder's Radius:", radius)
    print("Maximum number of inliers = ",maxinliers)
    print("Axis of cylinder = ", caxis)


    return [center, caxis, radius]


def q4_a(M: np.ndarray, D: np.ndarray) -> np.ndarray:
    '''
    Find transformation T such that D = T @ M. This assumes that M and D are
    corresponding (i.e. M[i] and D[i] correspond to same point)

    Attributes
    ----------
    M : np.ndarray
        Nx3 matrix of points
    D : np.ndarray
        Nx3 matrix of points

    Returns
    -------
    T : np.ndarray
        4x4 homogenous transformation matrix

    Hint
    ----
    use `np.linalg.svd` to perform singular value decomposition
    '''
    #initialize the transformation to be an identity matrix
    T = np.eye(4)
    #Center both the point clouds to be at the origin
    mm = np.mean(M,axis=0)
    dd = np.mean(D,axis=0)
    W = np.zeros((3, 3))
    #Iterate over the point cloud points
    for j in range(M.shape[0]):
            W += np.outer(M[j] - mm, D[j]- dd)
    u, s, vh = np.linalg.svd(W, full_matrices=True)
    R = np.matmul(u,vh)
    t = mm-np.matmul(R,dd)
    #Set the 3x3 top left matrix of the 4x4 transformation to be the rotation
    T[0:3,0:3] = R
    #Set the 3x1 right matrix of the 4x4 transformation to be the translation
    T[0:3,3] = t
    #Take inverse of current matrix to get transformation from D to M
    T = np.linalg.inv(T)
    return T


def q4_c(M: np.ndarray, D: np.ndarray) -> np.ndarray:
    '''
    Solves iterative closest point (ICP) to generate transformation T to best
    align the points clouds: D = T @ M

    Attributes
    ----------
    M : np.ndarray
        Nx3 matrix of points
    D : np.ndarray
        Nx3 matrix of points

    Returns
    -------
    T : np.ndarray
        4x4 homogenous transformation matrix

    Hint
    ----
    you should make use of the function `q4_a`
    '''
    #Initialize the transformation matrix to be a 4x4 identity matrix first
    T = np.eye(4)
    num_iterations = 1000
    #splitting the initial translation component
    t = T[0:3,3]
    #splitting the initial rotation component
    R= T[0:3,0:3]
    tolerance = 1e-6
    for i in range(num_iterations):
        index = np.zeros(M.shape[0],)
        for j in range(M.shape[0]):
            differences = D - M[j]
            distances = np.linalg.norm(differences, axis=1)
            index[j] = np.argmin(distances)
        indices = index.astype(int)
        M_1 = D[indices,:]
        #Use q4_a to calculate transformation between ICP iteration point cloud
        #and initial point cloud
        T_i = q4_a(M, M_1)
        #  Apply transformation to M
        M = np.dot(M, T_i[:3,:3].T)+T_i[0:3,3]
        T = np.dot(T_i, T)
        #splitting the transaltion component from current iteration
        t_new = T_i[0:3,3]
        #splitting the rotation component from current iteration
        R_new = T_i[0:3,0:3]
        #if the convergence is within the specified tolerance parameter, stop
        #icp algorithm iterations and return current converged tranformation
        if np.abs(t_new - t).max() < tolerance and np.abs(R_new - R).max() < tolerance:
            break

    return T

