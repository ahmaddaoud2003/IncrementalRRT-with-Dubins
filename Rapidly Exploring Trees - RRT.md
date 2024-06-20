# Rapidly Exploring Trees
Rapidly Exploring Tree is a sampling-based algortihm used in motion planning.
A sampling-based algortihm is an algorithm in which radnom nodes will be generated, and then plotted.

## Rapidly Exploring Trees - RRT:
  In this algorithm, nodes will be randomly generated and then they will be connected to the closest possilbe node. This process will continue till a node is generated in the proximity of the target or end point. 
  A viable path will be produced, but not an optimal one.

## Rapidly Exploring Trees Star - RRT*:
  For a randomly generated node in RRT*, the node will not be connected to the nearest node, instead, the algorithm will search within a specified search radius of the generated node. If nodes are present in the search radius, the algorithm will determine if the generated node can be connected to a seperate node which will minimize the path length. This way an optimal path will be found.

## The figure below will illustrate the difference
![RRT](https://github.com/ahmaddaoud2003/IncrementalRRT-with-Dubins/assets/145913339/df6706f1-87ee-4f6d-b47b-11c5c5d1096c)

## Incremental RRT:
   Incremental RRT improves upon the basic RRT algorithm by allowing for the addition of new nodes and edges to the existing tree. This means that instead of starting from scratch for each planning request, the algorithm can reuse parts of the existing tree that are still relevant.
   
A. Choudhary, ‘Sampling-based Path Planning Algorithms: A Survey’, arXiv [cs.RO]. 2023
