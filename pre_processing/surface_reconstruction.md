# Surface Reconstruction from Point Clouds
## 1) Important terms:
1. __Modeling__: the (mathematical) construction and computer implementation of an object, by defining
points in a 3 dimensional array. This array is based on the X, Y and Z axis of geometric space. Then,
different sets of these points are mathematically ’joined’ by say, lines, to create polygons and the
polygons joined to create objects. The simplest result is usually displayed as a wireframe model.
2. __Rendering__: is referred to the usually realistic drawing of 3D objects using computer technologies. In
order to render an object, certain properties like transparency, colour, diffuse or specular reflection and
refraction must be assigned to it and to the surrounding environment.
3. __Mesh__: it is a collection of triangular (or quadrilateral) contiguous, non-overlapping faces joined together
along their edges. A mesh therefore contains vertices, edges and faces and its easiest representation is a
single face. Sometimes it is also called TIN, Triangulated Irregular Network

## 2) Main steps involved in any algorithm are: 
![image1](../assets/Screenshot%20from%202022-09-06%2000-22-34.png)

## 3) Nearest neighbour search problem:
* Given a set S of points in a n-dimensional space, construct
a data structure which given any query point Q finds the point in S with the smallest distance to Q.
![image2](../assets/Screenshot%20from%202022-09-06%2000-24-45.png)

* We generally get a large set of data points in the point cloud data set which we receive from the sensor. Processing such a large data becomes tedious and is also unnecessary as there are many unwanted data points as well, so we need to reduce the point data so as to get accurate 3D structure. This can be acheived by the nearest neighbour search problem where we utilise the data points nearest to our query point and removing all the other unwanted point data thereby making our algorithms more efficient.
* The easiest approach to find the nearest neighbor
to the point Q will be to compute the distance
from Q to each point in P. This linear scan
approach is tolerable for small data sets but for
large data sets of point clouds, it is very
expensive. So we are using Kd-Trees.
* The key point of the problem formulation is that
dataset S is considered fixed. The query point may
vary from request to request Kdtree preprocesses the dataset and
build the tree data structure which accelerates
processing.
* The Kd-trees: 
    * Kd tree or a k-dimensional tree is a space-partitioning data structure for organizing points in a k-
dimensional space.
    * The k-d tree is a generalization of binary search trees in which every node is a k-
dimensional point.
    * Each node in the tree is defined by a plane through one of the dimensions that partitions the set of
points into left/right (or up/down) sets, each with half the points of the parent node. These children are
again partitioned into equal halves, using planes through a different dimension. Partitioning stops after
log n levels, with each point in its own leaf cell. The partitioning loops through the different dimensions
for the different levels of the tree, using the median point for the partition. kd-trees are known to work
well in low dimensions but seem to fail as the number of dimensions increase beyond three.
    * ![image3](../assets/Screenshot%20from%202022-09-06%2000-34-15.png)
    * Algorithm: 
        1) locate the location where the point would be located if it were added to the tree :
Starting with the root node, the algorithm moves down the tree recursively, taking decisions to
follow left or right node depending on whether the point is less than or greater than the current
node in the split dimension.
      2) Once the algorithm reaches a leaf node, it saves that node point as the "current best". As the tree is
traversed the distance between the point and the current node should be recorded.
      3) The algorithm goes back up the tree evaluating each branch of the tree that could have points
within the current minimum distance i.e. it unwinds the recursion of the tree, performing the
following steps at each node:
a) If the current node is closer than the current best, then it becomes the current best.
b) Check the other side of the hyperplane for points that could be closer to the search point than
the current best. To check this, intersect the splitting hyperplane with a hypersphere around the
search point that has a radius equal to the current nearest distance.
i) If the hypersphere crosses the plane, there could be nearer points on the other side of the
plane. The process is repeated in this branch.
ii) If the hypersphere doesn't intersect the splitting plane, then the algorithm continues
walking up the tree, and the entire branch on the other side of that node is eliminated.
      4) Search completes when the algorithm reaches the root node.


