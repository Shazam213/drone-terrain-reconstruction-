# Kd Trees And Nearest neighbour search porblem

* Type of a binary search tree, has additional constraints imposed on it.
* Space partitioning data structure for organizing points in a K-Dimensional space.
* very useful for range and nearest neighbor searches.
*  At the root of the tree all children will be split based on the first dimension (i.e. if the first dimension coordinate is less than the root it will be in the left-sub tree and if it is greater than the root it will obviously be in the right sub-tree). Each level down in the tree divides on the next dimension, returning to the first dimension once all others have been exhausted.
*  How to determine if a point will lie in the left subtree or in right subtree?
   *  If the root node is aligned in planeA, then the left subtree will contain all points whose coordinates in that plane are smaller than that of root node. Similarly, the right subtree will contain all points whose coordinates in that plane are greater-equal to that of root node.
* Creation of a 2-D Tree:
    
  *   Consider following points in a 2-D plane:
(3, 6), (17, 15), (13, 15), (6, 12), (9, 1), (2, 7), (10, 19)
  * Insert (3, 6): Since tree is empty, make it the root node.
  * Insert (17, 15): Compare it with root node point. Since root node is X-aligned, the X-coordinate value will be compared to determine if it lies in the right subtree or in the left subtree. This point will be Y-aligned.
  * Insert (13, 15): X-value of this point is greater than X-value of point in root node. So, this will lie in the right subtree of (3, 6). Again Compare Y-value of this point with the Y-value of point (17, 15) (Why?). Since, they are equal, this point will lie in the right subtree of (17, 15). This point will be X-aligned.
  * Insert (6, 12): X-value of this point is greater than X-value of point in root node. So, this will lie in the right subtree of (3, 6). Again Compare Y-value of this point with the Y-value of point (17, 15) (Why?). Since, 12 < 15, this point will lie in the left subtree of (17, 15). This point will be X-aligned.
  * Insert (9, 1):Similarly, this point will lie in the right of (6, 12).
  * Insert (2, 7):Similarly, this point will lie in the left of (3, 6).
  * Insert (10, 19): Similarly, this point will lie in the left of (13, 15).
    ![image1](/assets/Screenshot%20from%202022-09-10%2015-20-51.png)
    
* How is space partitioned?
  
    All 7 points will be plotted in the X-Y plane as follows:

    * Point (3, 6) will divide the space into two parts: Draw line X = 3.
      ![image2](../assets/Screenshot%20from%202022-09-10%2015-41-37.png)
    * Point (2, 7) will divide the space to the left of line X = 3 into two parts horizontally.
Draw line Y = 7 to the left of line X = 3.
      ![image3](/assets/Screenshot%20from%202022-09-10%2015-43-11.png)
  * Point (17, 15) will divide the space to the right of line X = 3 into two parts horizontally.
Draw line Y = 15 to the right of line X = 3.
    ![image4](/assets/Screenshot%20from%202022-09-10%2015-44-16.png)
  * Point (6, 12) will divide the space below line Y = 15 and to the right of line X = 3 into two parts.
Draw line X = 6 to the right of line X = 3 and below line Y = 15.
  ![image5](/assets/Screenshot%20from%202022-09-10%2015-45-12.png)
  * Point (13, 15) will divide the space below line Y = 15 and to the right of line X = 6 into two parts.
Draw line X = 13 to the right of line X = 6 and below line Y = 15.
  ![image6](/assets/Screenshot%20from%202022-09-10%2015-46-02.png)
  * Point (9, 1) will divide the space between lines X = 3, X = 6 and Y = 15 into two parts.
Draw line Y = 1 between lines X = 3 and X = 13.
  ![image7](../assets/Screenshot%20from%202022-09-10%2015-46-57.png)
  * Point (10, 19) will divide the space to the right of line X = 3 and above line Y = 15 into two parts.
Draw line Y = 19 to the right of line X = 3 and above line Y = 15.
  ![image8](/assets/Screenshot%20from%202022-09-10%2015-48-27.png)

* For simple codes pf addition, deletion and search refer : https://www.geeksforgeeks.org/k-dimensional-tree/

* Nearest Neighbour Search Algorithm:
  1)  locate the location where the point would be located if it were added to the tree :
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

Video link to visualize nearest neighbour search: https://www.youtube.com/watch?v=oQQrxiJvnhw

Also for the code refer: https://pcl.readthedocs.io/projects/tutorials/en/master/kdtree_search.html#kdtree-search 
