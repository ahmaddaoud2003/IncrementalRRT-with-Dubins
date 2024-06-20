#RRT algorithm
import numpy as np
import random

#Importing necessary function from previous modules
from ComputingDubinsCruves import norm

# A node class
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
        
#RRT Algorithm class
class RRTAlgorithm():
    def __init__(self, start, goal, numIterations, grid, turnRadius):
        self.randomTree = treeNode(start[0], start[1], 0)   #The RRT (root position) and heading
        self.goal = treeNode(goal[0], goal[1], 0)           #goal position and heading
        self.turnRadius = turnRadius                                #Dubins path turn radius
        self.nearestNode = None                                     #nearest node            
        self.iterations = min(numIterations, 200)                   #number of iterations to run
        self.grid = grid                                            #the map
        self.rho = 5*self.turnRadius                                #length of each branch 
        self.path_distance = 0                                      #total path distance  
        self.nearestDist = 10000                                    #distance to nearest node (initialize with large)
        self.numWaypoints = 0                                       #number of waypoints
        self.Waypoints = []                                         #the waypoints
        self.xPathTrajectory = []                                   #x trajectory of entire path
        self.yPathTrajectory = []                                   #y trajectory of entire path        

    def addChild(self, locationX, locationY, heading, distanceToParent, xTP, yTP):
        """
        Add a child node to the tree
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

    def sampleAPoint(self):
        """
        Sample a random point on the grid
        """
        x = random.randint(1, self.grid.shape[1])
        y = random.randint(1, self.grid.shape[0])
        point = np.array([x, y])
        return point

    def steerToPoint(self, locationStart, locationEnd):
        """
        Steering towards a point from starting point
        """
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= self.grid.shape[1]:
            point[0] = self.grid.shape[1] - 1
        if point[1] >= self.grid.shape[0]:
            point[1] = self.grid.shape[0] - 1
        if point[0] <= 0:
            point[0] = 0
        if point[1] <= 0:
            point[1] = 0
        return point

    def obstaclePresent(self, locationStart, locationEnd):
        """
        Check if the edge is intersected by an obstacle
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
        Check if the Dubins path is intersected by an obstacle
        """
        for i in range(len(xTrajectory)):
            if self.grid[round(min(self.grid.shape[0] - 1, yTrajectory[i])), round(min(self.grid.shape[1] - 1, xTrajectory[i]))] == 1:
                return True
        return False    

    def unitVector(self, locationStart, locationEnd):
        """
        Calculate the unit vector
        """
        v = np.array([locationEnd[0] - locationStart.locationX, locationEnd[1] - locationStart.locationY])
        u_hat = v/norm(v)
        return u_hat

    def findNearest(self, root, point):
        """
        find the nearest point to a given node
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
        cacluate the distacne
        """
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)         
        return dist

    def goalFound(self,point):
        """
        check if the goal is found
        """
        if self.distance(self.goal, point) <= self.rho:
            return True
        return False

    def resetNearestValues(self):
        """
        reset nearest node and distance values
        """
        self.nearestNode = None
        self.nearestDist = 10000

    def retraceRRTPath(self,goal):
        """
        retrace path from goal to start point
        """
        if goal.locationX == self.randomTree.locationX:
            return
        self.numWaypoints += 1
        currentPoint = np.array([goal.locationX, goal.locationY])
        self.Waypoints.insert(0,currentPoint)
        for i in reversed(range(len(goal.xTP))):
            self.xPathTrajectory.insert(0, goal.xTP[i])
            self.yPathTrajectory.insert(0, goal.yTP[i])
        self.path_distance += goal.parentDistance
        self.retraceRRTPath(goal.parent)