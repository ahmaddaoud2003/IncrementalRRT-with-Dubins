import numpy as np
import matplotlib.pyplot as plt
import random

#Importing necessary function from previous modules
from RRTStarDubins import RRTStarAlgorithm, treeNode
from ComputingDubinsCruves import atan2, r2d, returnDubinsPath

#Load the configuration space
grid = np.load('cspace.npy')
# Define start and end points
start = np.array([300.0, 300.0])
goal = np.array([1400.0, 775.0])
# Define parameters
numIterations = 300
turnRadius = 25
# Create a virtual representation of the goal region
goalRegion = plt.Circle((goal[0], goal[1]), 3*turnRadius, color='b', fill = False)

# Setting up the plot
fig = plt.figure("RRT Star Algorithm")
plt.imshow(grid, cmap='binary', origin = 'lower')
plt.plot(start[0],start[1],'ro') # Start point in red
plt.plot(goal[0],goal[1],'bo') #End goal in blue
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')


#Begin RRT* algorithm
rrtStar = RRTStarAlgorithm(start, goal, numIterations, grid, turnRadius)
plt.pause(2)

#start iterating
for i in range(rrtStar.iterations):
    rrtStar.resetNearestValues() # Reset nearest values for each iteration
    point = rrtStar.sampleAPoint() # Sample a new random point
    rrtStar.findNearest(rrtStar.randomTree, point) # find the nearest node in the tree to the sample point
    new = rrtStar.steerToPoint(rrtStar.nearestNode, point) # steer towards sampled point
    heading = atan2(goal[1] - new[1], goal[0] - new[0]) + random.random() # calculate a random heading towards goal
    
    print("Iteration: ",i, "Heading: ", r2d(heading))
    if not rrtStar.obstaclePresent(rrtStar.nearestNode, new): #Check if the path to the new point is obstacle free
        #Generate a Dubins' path towards the point
        xTP, yTP, pathDistance = returnDubinsPath(rrtStar.nearestNode.locationX, rrtStar.nearestNode.locationY, rrtStar.nearestNode.heading, new[0], new[1], heading, rrtStar.turnRadius)

        if pathDistance is not None and not rrtStar.dubinsObstaclePresent(xTP, yTP): # Chek if the dubins path is valid and obstacle-free
            rrtStar.findNeighbouringNodes(rrtStar.randomTree, new) # find neighboring nodes
            min_cost_node = rrtStar.nearestNode
            min_cost_xTrajectory = xTP
            min_cost_yTrajectory = yTP
            min_cost_distance = pathDistance
            min_cost = rrtStar.findPathDistance(min_cost_node)
            min_cost += pathDistance       
        else:
            continue
        # Check and update the minimum cost path
        for vertex in rrtStar.neighbouringNodes:
            vertex_cost = rrtStar.findPathDistance(vertex)
            xTP, yTP, pathDistance = returnDubinsPath(vertex.locationX, vertex.locationY, vertex.heading, new[0], new[1], heading, rrtStar.turnRadius)
            
            if pathDistance is not None and not rrtStar.dubinsObstaclePresent(xTP, yTP):
                vertex_cost += pathDistance
                if vertex_cost < min_cost:
                    min_cost_node = vertex
                    min_cost_xTrajectory = xTP
                    min_cost_yTrajectory = yTP
                    min_cost_distance = pathDistance
                    min_cost = vertex_cost

        # Update the nearest node and add the new node to the tree
        rrtStar.nearestNode = min_cost_node
        newNode = treeNode(new[0], new[1], heading)
        newNode.parentDistance = min_cost_distance
        newNode.xTP = min_cost_xTrajectory
        newNode.yTP = min_cost_yTrajectory
        rrtStar.addChild(newNode, rrtStar.nearestNode)

        # plot the new path segment
        plt.pause(1)
        plt.plot(min_cost_xTrajectory, min_cost_yTrajectory, 'k')
        plt.plot(new[0], new[1], marker="x", markersize=3, markeredgecolor="k", markerfacecolor="k")

        # rewire the tree if a better path is found
        for vertex in rrtStar.neighbouringNodes:
            vertex_cost = min_cost
            if vertex.locationX != newNode.locationX and vertex.locationY != newNode.locationY:
                xTP, yTP, pathDistance = returnDubinsPath(vertex.locationX, vertex.locationY, vertex.heading, new[0], new[1], heading, rrtStar.turnRadius)
                if pathDistance is not None and not rrtStar.dubinsObstaclePresent(xTP, yTP):
                    vertex_cost += pathDistance
                    if vertex_cost < rrtStar.findPathDistance(vertex):
                        vertex.parent = newNode
                        vertex.xTP = xTP
                        vertex.yTP = yTP
                        vertex.parentDistance = pathDistance
                        rrtStar.rewireCount += 1

        # check if the goal has been found
        point = np.array([newNode.locationX, newNode.locationY])
        if rrtStar.goalFound(point):
            xTP, yTP, pathDistance = returnDubinsPath(newNode.locationX, newNode.locationY, heading, goal[0], goal[1], 
                                                      0, rrtStar.turnRadius)
            

            if pathDistance is not None:
                projectedCost = rrtStar.findPathDistance(newNode) + pathDistance

                if projectedCost < rrtStar.goalCosts[-1]:
                    rrtStar.goal.parentDistance = pathDistance
                    rrtStar.goal.xTP = xTP
                    rrtStar.goal.yTP = yTP
                    rrtStar.addChild(rrtStar.goal, newNode)
                    plt.plot(xTP, yTP, 'k')
                    rrtStar.retracePath()
                    print("Goal Cost: ", rrtStar.goalCosts)
                    plt.pause(0.1)
                    rrtStar.Waypoints.insert(0,start)
                    for i in range(rrtStar.numWaypoints - 2):
                        plt.plot(rrtStar.Waypoints[i+1][0], rrtStar.Waypoints[i+1][1], marker="x", markersize=5, markeredgecolor="b", markerfacecolor="b")
                        plt.pause(0.01)
                    plt.plot(rrtStar.xPathTrajectory, rrtStar.yPathTrajectory, 'b')


# Annotate waypoint to the plot
for i in range(rrtStar.numWaypoints - 2): 
    text = str(round(rrtStar.Waypoints[i+1][0])) + ', ' + str(round(rrtStar.Waypoints[i+1][1]))
    plt.text(rrtStar.Waypoints[i+1][0] + 10, rrtStar.Waypoints[i+1][1] + 10, text, fontsize=6, fontweight="bold",color="r")
print("Number of waypoints: ", rrtStar.numWaypoints)
print("Minimum Path Distance (m): ", rrtStar.path_distance)     
plt.figure("Trajectory")
plt.imshow(grid, cmap='binary', origin = 'lower')
plt.plot(rrtStar.xPathTrajectory, rrtStar.yPathTrajectory)
for i in range(rrtStar.numWaypoints - 2):
    plt.plot(rrtStar.Waypoints[i+1][0], rrtStar.Waypoints[i+1][1], marker="x", markersize=7, markeredgecolor="b", markerfacecolor="b")
    text = str(round(rrtStar.Waypoints[i+1][0])) + ', ' + str(round(rrtStar.Waypoints[i+1][1]))
    plt.text(rrtStar.Waypoints[i+1][0] + 10, rrtStar.Waypoints[i+1][1] + 10, text, fontsize=7, fontweight="bold",color="r")
plt.plot(rrtStar.xPathTrajectory, rrtStar.yPathTrajectory, 'b')
plt.show()