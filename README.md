# A-Star-in-Python

This algorithm solves a maze by creating a graph, which is in the form of a python dictionary (or map) having keys as tuples(Cartesian coordinates of current position) and values as a vector of tuples(Cartesian coordinates of neighbours), from a 2D matrix of boolean values obtained from the (Prims) Maze Generator module. Since the hueristic used is Eucledian distance and distance between two neighbors is considered as 1 unit, the algorithm provides the shortest path to the destination.


![](https://github.com/VaibhavSaini19/A_Star-algorithm-in-Python/blob/master/astar.gif)
