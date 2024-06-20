#RRT algorithm
import numpy as np
import matplotlib.pyplot as plt
import random

##Importing necessary function from previous modules
from ComputingDubinsCruves import sqrt

# Node class
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
        
#RRT* Algorithm class
class RRTStarAlgorithm():
    def __init__(self, start, goal, numIterations, grid, turnRadius):
        self.randomTree = treeNode(start[0], start[1], 0)  #The RRT (root position) (has 0 cost)
        self.goal = treeNode(goal[0], goal[1], 0)          #goal position (initialize to a high cost)
        self.turnRadius = turnRadius                               #Dubins path turn radius
        self.nearestNode = None                                    #nearest node            
        self.iterations = min(numIterations, 400)                  #number of iterations to run
        self.grid = grid                                           #the map
        self.rho = 5*self.turnRadius                               #length of each branch   
        self.nearestDist = 10000                                   #distance to nearest node (initialize with large)
        self.numWaypoints = 0                                      #number of waypoints
        self.path_distance = 0                                     #total path distance  
        self.Waypoints = []                                        #the waypoints
        self.searchRadius = self.rho*2                             #the radius to search for finding neighbouring vertices 
        self.neighbouringNodes = []                                #neighbouring nodes  
        self.goalCosts = [10000]                                   #the costs to the goal (ignore first value)
        self.rewireCount = 0                                       #number of times the tree was rewired
        self.xPathTrajectory = []                                  #x trajectory of entire path
        self.yPathTrajectory = []                                  #y trajectory of entire path        
            

    def addChild(self, treeNode, nodeParent):
        """
        Add child node to the tree
        """
        if (treeNode.locationX == self.goal.locationX):
            nodeParent.children.append(self.goal)
            self.goal.parent = nodeParent
        else:    
            self.nearestNode.children.append(treeNode)
            treeNode.parent = self.nearestNode
        

    def sampleAPoint(self):
        """
        sampling a random point within the grid
        """
        x = random.randint(1, self.grid.shape[1])
        y = random.randint(1, self.grid.shape[0])
        point = np.array([x, y])
        return point
    

    def steerToPoint(self, locationStart, locationEnd):
        """
        Steering towards a point from staring point
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
        Check if the dubins path is intersected by an obstacle
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
        u_hat = v/np.linalg.norm(v)
        return u_hat
    

    def findNearest(self, root, point):
        """
        find the nearest node to a given point, update it recursively
        """
        if not root:
            return
        dist = self.distance(root, point)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children:
            self.findNearest(child, point)
    

    def findNeighbouringNodes(self, root, point):
        """
        find neighboring nodes within a radius
        """
        if not root:
            return
        dist = self.distance(root, point)
        if dist <= self.searchRadius and dist > 10:
            self.neighbouringNodes.append(root)
        for child in root.children:
            self.findNeighbouringNodes(child, point)        


    def distance(self, node1, point):
        """
        Calculate the distance
        """
        dist = sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)         
        return dist

    def goalFound(self, point):
        """
        Check if the goal is found
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
        self.neighbouringNodes = []
    

    def retracePath(self):
        """
        Retrace the path from the goal to the start
        """
        self.path_distance = 0
        self.numWaypoints = 0
        self.Waypoints = 0
        self.xPathTrajectory = []
        self.yPathTrajectory= []
        goal = self.goal
        while goal.locationX != self.randomTree.locationX: #update this
            self.numWaypoints += 1
            currentPoint = np.array([goal.locationX, goal.locationY])
            self.Waypoints.insert(0, currentPoint)
            for i in reversed(range(len(goal.xTP))):
                self.xPathTrajectory.insert(0, goal.xTP[i])
                self.yPathTrajectory.insert(0, goal.yTP[i])
            self.path_distance += goal.parentdistance
            goal = goal.parent
        self.goalCosts.append(self.path_distance)    

    def findPathDistance(self, node):
        """
        Calculate the path distance from the root to a given node
        """
        costFromRoot = 0
        currentNode = node
        while currentNode.locationX != self.randomTree.locationX:
            costFromRoot += currentNode.parentDistance
            currentNode = currentNode.parent #update this
            if currentNode.locationY == node.locationX and currentNode.locationY == node.locationY: #update this
                print('cycle detected!')
                break;
        return costFromRoot