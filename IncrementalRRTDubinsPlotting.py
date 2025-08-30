#RRT for driving algorithm
import numpy as np
import matplotlib.pyplot as plt
import random
mod = np.mod
#Importing required functions from previous modules
from ComputingDubinsCruves import atan2, returnDubinsPath
from IncrementalRRTDubins import IncrementalRRT


#Loading the roadmap grid for the configuration space
grid = np.load('cspaceRoadmap.npy')
fig = plt.figure("RRT Driving Algorithm")
plt.imshow(grid, cmap='binary', origin = 'lower')


def findNearTarget(i):
    xLocation = [10, 250, 500, 750, 1000, 1250, 1500, 1750, 1990]
    yLocation = [600, 365, 189, 254, 425, 675, 725, 675, 425]
    headingSetpoint = [-0.8, -0.8, 0, 0.8, 0.8, 0.8, 0, -0.8, -0.8]
    return xLocation[i], yLocation[i], headingSetpoint[i]


X = 5
Y = 600
start = np.array([X,Y])
psi = -0.7854
turnRadius = 25
numIterations = 50
rrTrees = []

'''
This is the backbone of the Incremental part.
We set the goal as the next corresponding points the Location arrays
When we reach the target it becomes the the new starting point and target point becomes the next one, we repaet iteratively until the end
'''
for i in range(8):
    xTarget, yTarget, psiTarget = findNearTarget(i+1)
    plt.plot(xTarget, yTarget, marker="x", markersize=8, markeredgecolor="g", markerfacecolor="g")
    start = np.array([X,Y])
    goal = np.array([xTarget, yTarget])

    #Initializing the RRT algorithm
    rrt = IncrementalRRT(start,goal,numIterations, grid, turnRadius, psiTarget)

    #Iterations for RRT
    for k in range (rrt.iterations):
        rrt.resetNearestValues()
        point = rrt.sampleNewPoint(X, Y, psi)
        rrt.findNearest(rrt.randomTree, point)
        new = rrt.steerToPoint(rrt.nearestNode, point)
        heading = atan2(goal[1] - new[1], goal[0] - new[0]) + random.uniform(-1, 1)

        #Checking if new point falls in an obstacle
        if not rrt.obstaclePresent(rrt.nearestNode, new):
            xTP, yTP, pathDistance = returnDubinsPath(rrt.nearestNode.locationX, rrt.nearestNode.locationY, rrt.nearestNode.heading, new[0], new[1],heading, rrt.turnRadius)
            if pathDistance is not None and not rrt.dubinsObstaclePresent(xTP, yTP):
                rrt.addChild(new[0], new[1], heading, pathDistance, xTP, yTP)
                plt.pause(0.01)
                plt.plot(xTP,yTP,'k')
                plt.plot(new[0], new[1], marker="x", markersize=3, markeredgecolor="k", markerfacecolor="k")
                
                xTP, yTP, pathDistance = returnDubinsPath(new[0], new[1], heading, goal[0], goal[1], psiTarget, rrt.turnRadius)
                
                if pathDistance is not None and not rrt.dubinsObstaclePresent(xTP, yTP):
                    projectedCost = rrt.findPathDistance(rrt.nearestNode.children[-1]) + pathDistance
                    
                    if projectedCost < rrt.goalCosts[-1]:
                        rrt.nearestNode = rrt.nearestNode.children[-1]
                        rrt.addChild(goal[0], goal[1], 0, pathDistance, xTP, yTP)
                        rrt.retracePath()
                        plt.plot(xTP, yTP, 'k') 
                        
    plt.plot(rrt.xTrajectory, rrt.yTrajectory,"b")
    rrt.goalCosts.pop(0)
    rrTrees.append(rrt)
    X = xTarget
    Y = yTarget  
    psi = psiTarget                     


#Combine final trajectories and calculate total distance
xFinalTrajectory = []
yFinalTrajectory = []
totalDistance = 0
for i in range(8):
    for j in range(len(rrTrees[i].xTrajectory)):
        xFinalTrajectory.append(rrTrees[i].xTrajectory[j])
        yFinalTrajectory.append(rrTrees[i].yTrajectory[j])
    print (rrTrees[i].goalCosts)
    totalDistance += rrTrees[i].path_distance
    
print("Total Distance", totalDistance)
plt.figure("Final Trajectory")
grid = np.load('cspaceRoadmap.npy')
plt.imshow(grid, cmap='binary', origin = 'lower')
plt.plot(xFinalTrajectory, yFinalTrajectory)    
plt.show()
