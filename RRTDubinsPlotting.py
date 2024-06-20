import numpy as np
import matplotlib.pyplot as plt
import random
#Importing necessary function from previous modules
from RRTDubinsClass import RRTAlgorithm
from ComputingDubinsCruves import atan2, r2d, returnDubinsPath

#load configurration space
grid = np.load('cspace.npy')
#Define start and end points
start = np.array([100.0, 100.0])
goal = np.array([1600.0, 600.0])
#Define parameters
numIterations = 85
turnRadius = 25
# Create a virtual representation of the goal region
goalRegion = plt.Circle((goal[0], goal[1]), 3*turnRadius, color='b', fill = False)

# Setting up the plot
fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='binary', origin = 'lower')
plt.plot(start[0],start[1],'ro') # Start point in red
plt.plot(goal[0],goal[1],'bo') #End goal in Blue
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')
 


#Begin RRT algorithm
rrt = RRTAlgorithm(start, goal, numIterations, grid, turnRadius)
plt.pause(2)

# Start iterating
for i in range(rrt.iterations):
    rrt.resetNearestValues() #reset nearest values for each iteration
    point = rrt.sampleAPoint() # Samle a new random point
    rrt.findNearest(rrt.randomTree, point) # find the nearest point in the tree to the sampled point
    new = rrt.steerToPoint(rrt.nearestNode, point) #Steer towards a sampled point
    heading = atan2(goal[1] - new[1], goal[0]-new[0]) + random.random() #Calculate a random heading towards goal
    print("Iteration: ",i, "Heading: ", r2d(heading))
    
    if not rrt.obstaclePresent(rrt.nearestNode, new): #Check if the path is obstacle-free
        #Generate a Dubins curve
        xTP, yTP, pathDistance = returnDubinsPath(rrt.nearestNode.locationX, rrt.nearestNode.locationY, rrt.nearestNode.heading, new[0], new[1], heading, rrt.turnRadius)
        
        if pathDistance is not None and not rrt.dubinsObstaclePresent(xTP, yTP): #Check if the dubins path is valid and obstacle-free
            rrt.addChild(new[0], new[1], heading, pathDistance, xTP, yTP)
            plt.pause(0.5)
            plt.plot(xTP,yTP,'k')
            plt.plot(new[0], new[1], marker="x", markersize=3, markeredgecolor="k", markerfacecolor="k")
            
        if rrt.goalFound(new): #Check if the goal has been found
            xTP, yTP, pathDistance = returnDubinsPath(rrt.nearestNode.locationX, rrt.nearestNode.locationY, rrt.nearestNode.heading, goal[0], goal[1], 0, rrt.turnRadius)
            
            if pathDistance is not None and not rrt.dubinsObstaclePresent(xTP, yTP):
                rrt.addChild(goal[0], goal[1], 0, pathDistance, xTP, yTP)
                plt.plot(xTP,yTP,'k')
                rrt.retraceRRTPath(rrt.goal)
                
                # Display route
                rrt.Waypoints.insert(0,start)
                print("Goal found!")
                print("Number of waypoints: ", rrt.numWaypoints)
                print("Path Distance (m): ", rrt.path_distance) 
                plt.plot(rrt.xPathTrajectory, rrt.yPathTrajectory, 'b')
                for i in range(rrt.numWaypoints - 2):
                    plt.plot(rrt.Waypoints[i+1][0], rrt.Waypoints[i+1][1], marker="x", markersize=5, markeredgecolor="b", markerfacecolor="b")
                    text = str(round(rrt.Waypoints[i+1][0])) + ', ' + str(round(rrt.Waypoints[i+1][1]))
                    plt.text(rrt.Waypoints[i+1][0] + 10, rrt.Waypoints[i+1][1] + 10, text, fontsize=6, fontweight="bold",color="r")
                break
            else:
                print('no path found')

# Plot the final trajectory and waypoint
plt.figure("Trajectory")
plt.imshow(grid, cmap='binary', origin = 'lower')
plt.plot(rrt.xPathTrajectory, rrt.yPathTrajectory, 'b')
for i in range(rrt.numWaypoints - 2):
    plt.plot(rrt.Waypoints[i+1][0], rrt.Waypoints[i+1][1], marker="x", markersize=5, markeredgecolor="b", markerfacecolor="b")
    text = str(round(rrt.Waypoints[i+1][0])) + ', ' + str(round(rrt.Waypoints[i+1][1]))
    plt.text(rrt.Waypoints[i+1][0] + 10, rrt.Waypoints[i+1][1] + 10, text, fontsize=7, fontweight="bold",color="r")            
plt.xlim([0,grid.shape[1]])
plt.ylim([0,grid.shape[0]])
plt.show()