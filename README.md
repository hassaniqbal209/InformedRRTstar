In this work, my group implemented Informed RRT* and showed results in various scenarios of a car navigating in heavily cluttered environment. 

Informed RRT star builds on regular RRT star, which builds on RRT. The regular RRT algorithm, which means rapidly exploring random tree, begins by drawing a uniformly distributed random sample from the map space, which is this red dot. Then, if there are already nodes in the tree, it finds the closest one, which is this green dot.  Then if the new random node is too far away, or violates the vehicle constraints, the node location is moved to a better “more reasonable” location, making sure this new location does not collide with any obstacles.  This new node is then added to the tree, with the nearest node being its parent. This algorithm is simple and prob complete, but it is not optimal, and definitely not efficient.

RRT* algorithm builds on RRT by adding 2 new features: 
1. considering all nodes within a neighborhoods of the random sample, not just the nearest one
2. Rewiring of the tree structure within this neighborhood. 

RRT* is a probabilistically optimal algorithm, which is a huge benefit over RRT. This also makes RRT* more efficient. RRT* asymptotically finds optimal paths from the start to the goal in the planning problem but it does this by finding optimal paths from the start to every other state in the planning domain. This is very inefficient. 

Ideally, we want to limit our search to a sub planning problem that contains all possibly better solutions. This will improve the convergence rate of RRT* as well as increase probability of finding difficult passages in finite time. 

We do this by uniformly sampling from space inside an ellipse defined by path length, start and goal location. We will come to how we achieve this sampling in a while. It can be shown that once an initial solution is found with regular RRT*, at any iteration of RRT*, all possible improvements are contained in this ellipse i.e. it’s necessary that sample drawn to improve the solution must lie in the ellipse. So by directly sampling from the ellipse area, we focus the search to only the states that have the possibility of improving the solution. This increases solution improvement rate causing the ellipse to quickly shrink and further focus the search for improvements/ This approach can be generalized to n-dimensions and makes the Informed RRT* effective for even higher dimensions. 

Informed RRT* is a simple and computationally inexpensive solution to improve convergence rate of RRT* regardless of state dimension. It takes order of magnitude fewer iterations than RRT* with no practical increase in computational cost. For a max number of iterations, Informed RRT* is able to find solutions with tighter constraints of the optimum. In absence of obstacles, able to find the optimal path in finite time.


### Clone and Build
1. `mkdir projects`
1. `cd projects`
1. `git clone <your repository url>` (found in the upper right)
1. `cd <cloned_repo>`
1. `make -j`
