#Dubin's path 
import numpy as np
import matplotlib.pyplot as plt
import math

# Unnecessary shortcuts for numpy functions and constant (simplifies the code)
cos = np.cos
sin = np.sin
norm = np.linalg.norm
atan2 = np.arctan2
r2d = np.rad2deg
cross = np.cross
atan = math.atan
sqrt = np.sqrt
pi = np.pi
acos = math.acos


#return Dubin's path
def returnDubinsPath(x1, y1, psi1, x2, y2, psi2, R):
    # find the unit vector of initial and final headings
    u_vec1 = np.array([cos(psi1), sin(psi1)])
    u_vec2 = [x2-x1, y2-y1]
    u_vec2 = u_vec2/norm(u_vec2)
    u_vec3 = np.array([cos(psi2), sin(psi2)])

    # calculate cross products to determine turn types
    cp1 = cross(u_vec1, u_vec2)
    cp2 = cross(u_vec2, u_vec3)
    
    # determine turn types (CSC or CCC) based on cross products
    sigma1, sigma2, turnType = findCSCTurnType(cp1, cp2)
    xc1, yc1 = returnCircleCenter(x1, y1, psi1, sigma1, R)
    xc2, yc2 = returnCircleCenter(x2, y2, psi2, sigma2, R)
    
    #Check the distance between circle centers to determine trajectory type
    if distance(xc1, xc2, yc1, yc2) >= 3*R:
        xtrajectory, ytrajectory, pathDistance = computeCCCTrajectory(x1, x2, xc1, xc2, y1, y2, yc1, yc2, psi1, psi2, R, turnType, sigma1, sigma2)
    elif distance(xc1, xc2, yc1, yc2) < 3*R and distance(xc1, xc2, yc1, yc2) >= 2*R:
        sigma1, sigma2, turn, proceed = findCCCTurnType(turnType)
        if proceed: 
            xtrajectory, ytrajectory, pathDistance = computeCSCTrajectory(x1, x2, xc1, xc2, y1, y2, yc1, yc2, R, sigma1, sigma2)
        else:
            xtrajectory = None 
            ytrajectory = None
            pathDistance = None
    else:
        xtrajectory = None 
        ytrajectory = None       
        pathDistance = None
    return xtrajectory, ytrajectory, pathDistance
        
        
#Determine CSC turn types based on cross products
def findCSCTurnType(cp1, cp2):
    if (cp1 <= 0 and cp2 > 0):
        turn = "RSL"
        sigma1 = -1
        sigma2 = 1
        
    elif (cp1 <= 0  and cp2 < 0):
        turn = "RSR"
        sigma1 = -1
        sigma2 = -1
        
    elif (cp1 > 0 and cp2 < 0):
        turn = "LSR"
        sigma1 = 1
        sigma2 = -1
        
    elif (cp1 > 0 and cp2 > 0):
        turn = "LSL"
        sigma1 = 1
        sigma2 = 1
    return sigma1, sigma2, turn

#Determine CCC turn types based on cross product
def findCCCTurnType(turnOld):
    proceed = False
    sigma1 = 0
    sigma2 = 0
    turn = turnOld
    if turnOld[0] == turnOld[2]:
        if turnOld[0] == "R":
            turn = "RLR"
            sigma1 = -1
            sigma2 = -1
        elif turnOld[0] == "L":    
            turn = "LRL"
            sigma1 = 1
            sigma2 = 1
        proceed = True
    return sigma1, sigma2, turn, proceed

# Return the center of turning circle
def returnCircleCenter(x, y, psi, sigma, R):
    #update xc and yc
    xc = x + R * cos(psi + sigma * pi / 2)
    yc = y + R * sin(psi + sigma * pi / 2)
    return xc, yc

#Calculate the euclidean distance between the two centers
def distance(x1, x2, y1, y2):
    return sqrt((y2-y1)**2 + (x2-x1)**2)

#Compute the CSCtrajectory
def computeCSCTrajectory(x1, x2, xc1, xc2, y1, y2, yc1, yc2, psi1, psi2, R, turnType, sigma1, sigma2):
    #find heading and distance
    psiL = atan2(yc2-yc1, xc2-xc1)    
    Stan = distance(xc1, xc2, yc1, yc2)

    # find intermediate heading angle based on angle psiD based on turn type
    if turnType == "RSR" or turnType == "LSL":
        psiD = psiL
    elif turnType == "LSR":
        psiD = psiL + atan(2*R/Stan)
        if psiD > pi:
            psiD = psiL - 2*pi
    elif turnType == "RSL":
        psiD = psiL - atan(2*R/Stan)
        if psiD < -pi:
            psiD = 2 * pi - abs(psiD)
        
    #initialize path distance and trejectory lists
    pathDistance = 0
    xt1 = [x1]
    yt1 = [y1]
    xco = [x1]
    yco = [y1]
    psiT = psi1
    i = 1

    # calculate the trajectory for the first turn segment
    while (not psiD - 0.1 < psiT < psiD + 0.1):
        i = i + 1
        Mco = np.array([[cos(0.05*i),-sigma1*sin(0.05*i)],[sigma1*sin(0.05*i),cos(0.05*i)]])
        vec1 = np.array([[x1 - xc1],[y1 - yc1]])
        [xco, yco] = Mco @ vec1 + np.array([[xc1],[yc1]])    
        xt1.append(xco[0])
        yt1.append(yco[0])

        # Update path distance and psiT
        pathDistance = pathDistance + R * (0.05)
        psiT = psiT + sigma1 * 0.05

        # Adjust psiT if it falls below -pi or exceeds pi
        if (psiT < -pi):
            psiT = 2 * pi - abs(psiT)
        if (psiT > pi):
            psiT = psiT - 2 * pi

    # calculate the straight segment trajectory between the two turns
    xt2 = [x2]
    yt2 = [y2]
    xci = [x2]
    yci = [y2]
    psiT = psi2
    i = 1
    # generate a trajectory from cut-in point to the end
    while (not psiD - 0.1 < psiT < psiD + 0.1):
        i = i + 1
        Mci = np.array([[cos(0.05*i),sigma2*sin(0.05*i)],[-sigma2*sin(0.05*i),cos(0.05*i)]])
        vec2 = np.array([[x2 - xc2],[y2 - yc2]])
        [xci, yci] =  Mci @ vec2 + np.array([[xc2],[yc2]])   
        xt2.insert(0,xci[0])
        yt2.insert(0,yci[0])
        pathDistance = pathDistance + R*(0.1)
        psiT = psiT - sigma2*0.05
        #Adjust psiT if it falls below -pi or exceeds pi
        if (psiT < -pi):
            psiT = 2 * pi - abs(psiT)
        if (psiT > pi):
            psiT = psiT - 2 * pi
    
    xtrajectory = xt1
    ytrajectory = yt1
    v = np.array([xci[0] - xco[0], yci[0] - yco[0]])
    u_hat = v/norm(v)
    dist = distance(xci[0], xco[0], yci[0], yco[0])
    # Generate straight trajectory between cut-out and cut-in points
    for j in range(int(dist/3)):
        xtrajectory.append(xtrajectory[-1] + 3*u_hat[0])
        ytrajectory.append(ytrajectory[-1] + 3*u_hat[1])
    # Append final trajectory segment
    for j in range(len(xt2)):
        xtrajectory.append(xt2[j])
        ytrajectory.append(yt2[j])  
    pathDistance = pathDistance + dist
    return xtrajectory, ytrajectory, pathDistance

# Compute CCC trajectory
def computeCCCTrajectory(x1, x2, xc1, xc2, y1, y2, yc1, yc2, R, sigma1, sigma2):
    Stan = distance(xc1, xc2, yc1, yc2)
    # Calculate intermediate circle center
    u_vec4 = np.array([[xc2-xc1], [yc2-yc1]])
    u_vec4 = u_vec4/norm(u_vec4)
    theta = acos(Stan/(4*R))
    #Calculate the intermediate circle center and initial circle center
    Mco = np.array([[cos(theta),-sigma1*sin(theta)],[sigma1*sin(theta),cos(theta)]])
    [xc3, yc3] = 2*R*(Mco @ u_vec4) + np.array([[xc1],[yc1]])
    [xco, yco] = R*(Mco @ u_vec4) + np.array([[xc1],[yc1]])
    #Calculate the second circle center and rotation
    Mci = np.array([[cos(theta),sigma2*sin(theta)],[-sigma2*sin(theta),cos(theta)]])
    [xci, yci] = -R*(Mci @ u_vec4) + np.array([[xc2],[yc2]])

    # Initialize trajectory with the starting point and initialize path distance
    xtrajectory = [x1]
    ytrajectory = [y1]
    pathDistance = 0

    # Turb towards cut-out point
    while distance(xtrajectory[-1], xco, ytrajectory[-1], yco) > 3:
        Mco = np.array([[cos(0.1),-sigma1*sin(0.1)],[sigma1*sin(0.1),cos(0.1)]])
        vec1 = np.array([[xtrajectory[-1] - xc1],[ytrajectory[-1] - yc1]])
        [xnew, ynew] = Mco @ vec1 + np.array([[xc1],[yc1]])
        xtrajectory.append(xnew[0])
        ytrajectory.append(ynew[0])  
        pathDistance = pathDistance + R * (0.1)
    # Append the cut-out point to the trajectory
    xtrajectory.append(xco[0])
    ytrajectory.append(yco[0]) 
    
    # Turn towards the middle circle
    while distance(xtrajectory[-1], xci, ytrajectory[-1], yci) > 3:
        #TODO: update Mci using sigma2 in opposite direction (0.1 rad intervals), vec2 (vector from <xc3, yc3> to latest point of xy_trajectory), and xnew, ynew
        Mci = np.array([[cos(0.1), sigma2*sin(0.1)],[-sigma2*sin(0.1),cos(0.1)]])
        vec2 = np.array([[xtrajectory[-1] - xc3[0]],[ytrajectory[-1] - yc3[0]]])
        [xnew, ynew] = Mci @ vec2 + np.array([[xc3[0]],[yc3[0]]])
        #TODO: append xnew, ynew to xy_trajectory, add 0.1*R to path distance
        xtrajectory.append(xnew[0])
        ytrajectory.append(ynew[0])  
        pathDistance = pathDistance + R * (0.1)
    #Append the middle circle cut-in point ot the trajectory
    xtrajectory.append(xci[0])
    ytrajectory.append(yci[0])
    
    # Turn towards the final destination
    while distance(xtrajectory[-1], x2, ytrajectory[-1], y2) > 3:
        Mc3 = np.array([[cos(0.1),-sigma1*sin(0.1)],[sigma1*sin(0.1),cos(0.1)]])
        vec3 = np.array([[xtrajectory[-1] - xc2],[ytrajectory[-1] - yc2]])
        [xnew, ynew] = Mc3 @ vec3 + np.array([[xc2],[yc2]])
        xtrajectory.append(xnew[0])
        ytrajectory.append(ynew[0])  
        pathDistance = pathDistance + R * (0.1)
    # Append the final destination point to the trajectory
    xtrajectory.append(x2)
    ytrajectory.append(y2)
    return xtrajectory, ytrajectory, pathDistance