import numpy as np
import matplotlib.pyplot as plt
import random

#Importing required functions from other modules
from ComputingDubinsCruves import atan2, norm, r2d, pi, cos, sin, returnDubinsPath

#Class which will represent a node in the RRT tree
class treeNode():
    def __init__(self, locationX, locationY, heading):
        self.locationX = locationX                #X Location
        self.locationY = locationY                #Y Location  
        self.heading = heading                    #Heading (rad)
        self.children = []                        #children list   
        self.parent = None                        #parent node reference 
        self.parentDistance = 0                   #distance to parent node
        self.xTP = None                           #x values of dubins path to parent
        self.yTP = None                           #y values of dubins path to parent
        
#Incremental RRT Algorithm class
class IncrementalRRT():
    def __init__(self, start, goal, numIterations, grid, turnRadius, psiTarget):
        self.randomTree = treeNode(start[0], start[1], 0)   #The RRT (root position) and heading
        self.goal = treeNode(goal[0], goal[1], psiTarget)           #goal position and heading
        self.turnRadius = turnRadius                                #Dubins path turn radius
        self.nearestNode = None                                     #nearest node            
        self.iterations = min(numIterations, 200)                   #number of iterations to run
        self.grid = grid                                            #the map (occipancy map)
        self.rho = 5*self.turnRadius                                #length of each branch (step size for tree expansion)
        self.path_distance = 0                                      #total path distance  
        self.nearestDist = 10000                                    #distance to nearest node (initialize with large)
        self.numWaypoints = 0                                       #number of waypoints
        self.Waypoints = []                                         #the waypoints
        self.xTrajectory = []                                       #x trajectory of entire path
        self.yTrajectory = []                                       #y trajectory of entire path
        self.goalCosts = [5000]                                     #cost to goal
        

    def addChild(self, locationX, locationY, heading, distanceToParent, xTP, yTP):
        """
        add a child node to the nearest node in the tree
        """
        if (locationX == self.goal.locationX):
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
            self.goal.parentDistance = distanceToParent
            self.goal.xTP = xTP
            self.goal.yTP = yTP
        else:    
            tempNode = treeNode(locationX, locationY, heading)
            self.nearestNode.children.append(tempNode)
            tempNode.parent = self.nearestNode
            tempNode.parentDistance = distanceToParent
            tempNode.xTP = xTP
            tempNode.yTP = yTP
        

    def sampleNewPoint(self, x, y, psi):
        """
        Generate random points around current location
        Random distance and random heading points
        """
        r0 = 200*np.random.random()  # random distance
        seq = [-1,1]
        s = random.sample(seq, 1)
        tht = psi + s[0]*np.random.random()*pi/2  # random angle
        xSample = x + r0*cos(tht)
        ySample = y + r0*sin(tht)
        return np.array([xSample, ySample])
    

    def steerToPoint(self, locationStart, locationEnd):
        """
        Steers towards a new point from the nearest node, while ensuring the point is within the grid boundaries
        Offset is how far and in what direction to place the next tree node (Help us move incrementally to the target)
        """
        offset = self.rho*self.unitVector(locationStart, locationEnd) 
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]]) 
        # Ensure the new point is within grid boundaries
        if point[0] >= self.grid.shape[1]:
            point[0] = self.grid.shape[1] - 1
        if point[1] >= self.grid.shape[0]:
            point[1] = self.grid.shape[0] - 1
        if point[0] <= 0:
            point[0] = 0
        if point[1] <= 1:
            point[1] = 1
        return point
    

    def obstaclePresent(self, locationStart, locationEnd):
        """
        Check if the edge between the two nodes intersects an obstacle
        """
        u_hat = self.unitVector(locationStart, locationEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.rho):
            testPoint[0] = min(self.grid.shape[1]-1,locationStart.locationX + i*u_hat[0])
            testPoint[1] = min(self.grid.shape[0]-1,locationStart.locationY + i*u_hat[1])
            if self.grid[round(testPoint[1]),round(testPoint[0])] == 1:
                return True
        return False
    

    def dubinsObstaclePresent(self, xTrajectory, yTrajectory):
        """
        Check if the Dubins' path intersects an obstacle
        """
        for i in range(len(xTrajectory)):
            if self.grid[round(min(self.grid.shape[0] - 1, yTrajectory[i])), round(min(self.grid.shape[1] - 1, xTrajectory[i]))] == 1:
                return True
        return False    

    def unitVector(self, locationStart, locationEnd):
        """
        Computing the unit vector from start to end point
        """
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = v/norm(v)
        return u_hat

    def findNearest(self, root, point):
        """
        Find the nearest node in the tree to the given point
        """
        if not root:
            return
        dist = self.distance(root, point)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children:
            self.findNearest(child, point)


    def distance(self, node1, point):
        """
        Computing distance
        """
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)         
        return dist
    

    def goalFound(self,point):
        """"
        Checks if a node has been created in the goal's proximity
        """
        if self.distance(self.goal, point) <= self.rho:
            return True
        return False
    

    def resetNearestValues(self):
        """
        Resets the nearest node and distance value to preprae for next iteration
        """
        self.nearestNode = None
        self.nearestDist = 10000
        

    def findPathDistance(self, node):
        """
        Computes the total distance from the root to the given node
        """
        costFromRoot = 0
        currentNode = node
        #Traverse the node from the node to the root, accumulating the distance
        while currentNode.locationX != self.randomTree.locationX:
            costFromRoot += currentNode.parentDistance
            currentNode = currentNode.parent

            if currentNode.locationX == node.locationX and currentNode.locationY == node.locationY:
                print('cycle detected!')
                break;

        return costFromRoot         

    def retracePath(self):
        """
        Retracing the path from the goal to root, updating the path distance, waypoints and trajectory.
        """
        self.path_distance = 0
        self.numWaypoints = 0
        self.Waypoints = []
        self.xTrajectory = []
        self.yTrajectory = []
        goal = self.goal
        
        while goal.locationX != self.randomTree.locationX:
            self.numWaypoints += 1
            currentPoint = np.array([goal.locationX, goal.locationY])
            self.Waypoints.insert(0,currentPoint)
            for i in reversed(range(len(goal.xTP))):
                self.xTrajectory.insert(0, goal.xTP[i])
                self.yTrajectory.insert(0, goal.yTP[i])
            self.path_distance += goal.parentDistance
            goal = goal.parent

        self.goalCosts.append(self.path_distance)
