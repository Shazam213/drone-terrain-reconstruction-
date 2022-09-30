<h1> Fast Triangulation & Associated Parameters </h1>

# Table of Contents
1. [Introduction](#Introduction)
2. [Defining Parameters](#Defining-Parameters)
3. [Main Algorithm](#Main-Algorithm)
4. [References](#References)

<h2>Introduction<a name="Introduction"></a></h2>

<!-- Blockquote -->
> The approach for creating the triangular mesh is based on the *incremental surface growing principle*. <br>
> Our algorithm selects a starting triangle’s vertices and connects new triangles to it until either all points are considered or no more valid triangles can be connected. In the second case a new seed triangle is placed in the unconnected part and the triangulation is restarted.

<h2> Defining Parameters<a name="Defining-Parameters"></h2>
  
* The `GreedyProjectionTriangulation` class of the Point Cloud Library is used for the implementation of the Fast Triangulation Algorithm, on 3D Points on the basis of local 2D Projections. This class inherits certain properties and methods, which can aptly be understood from the Inheritance Diagram below: <br>
![Inheritance-Diagram](https://user-images.githubusercontent.com/95737452/193227367-a44aceef-38d3-4319-b650-4540a87888a6.png) <br>
  
 * Following Parameters define the implementation of this algorithm:
  
  1. `nnn_`(unsigned int) and `mu_`(double): These parameters control the size of the neighborhood of projection. The former defines the maximum number of neighbors that are searched for, while the latter specifies the maximum acceptable distance for a point to be considered, relative to the distance of the nearest point (in order to adjust to changing densities). These parameters are referenced by the functions `setMaximumNearestNeighbors()` and `setMu()` respectively.
  2. `search_radius_` (double) is practically the maximum edge length for every triangle. This has to be set by the user such that to allow for the biggest triangles that should be possible. It is defined by the product of `mu_` and the distance of the nearest neighbour in the k-neighbourhood in order to accomodate varying point densities. Referenced by the function `setSearchRadius()`.
  3. `minimum_angle_` (double) and `maximum_angle_` (double) are the minimum and maximum angles in each triangle.
  4. `maximum_angle_` and `consistent_`(bool) are meant to deal with the cases where there are sharp edges or corners and where two sides of a surface run very close to each other. To achieve this, points are not connected to the current point if their normals deviate more than the specified angle (note that most surface normal estimation methods produce smooth transitions between normal angles even at sharp edges). This angle is computed as the angle between the lines defined by the normals (disregarding the normal’s direction) if the normal-consistency-flag is not set, as not all normal estimation methods can guarantee consistently oriented normals. Typically, 45 degrees (in radians) and false works on most datasets.

<h2>Main Algorithm <a name="Main-Algorithm"></a></h2>
  
* **STEP I**: Nearest neighbor search: 
  * For each point ‘p’ in the point cloud, a k-neighborhood is selected. This neighborhood is created by searching the reference point’s nearest k-neighbors within a sphere of radius r, previously defined as the product of mu and d. 
  * To find the nearest neighbors for the given point in the point cloud, Kd-tree nearest neighbor search has been used.
  * The points in the cloud are assigned various states depending on their interaction with the algorithm: free, fringe, boundary, and completed.
  > a. Initially, all the points in the cloud are in the free state and **free** points are defined as those points which have no incident triangles.<br>
  > b. When all the incident triangles of a point have been determined, the point is referred to as of **completed** state. <br>
  > c. When a point has been chosen as a reference point but has some missing triangles due to the maximum allowable angle parameter, it is referred to as a **boundary** point. <br>
  > d. **Fringe** points are the points that have not yet been chosen as a reference point

* **STEP II**: Neighborhood projection using tangent planes: 
  * The neighborhood is projected on a plane that is approximately tangential to the surface formed by the neighborhood and ordered around p, and triangulation follows.
  * Greedy Projection Triangulation works operates on 3D points based on local 2D projections, and assumes locally smooth surfaces and relatively smooth transitions between areas with different point densities.
  ![Greedy](https://user-images.githubusercontent.com/95737452/193234288-9af26a1a-822a-428c-802b-f59a1a9b0bf3.png)

<h2>References<a name="References"></a></h2>

1. Understanding:<br>
  https://ias.informatik.tu-muenchen.de/_media/spezial/bib/marton09icra.pdf <br>
  https://nccastaff.bmth.ac.uk/jmacey/OldWeb/MastersProjects/MSc13/14/Thesis/SurfaceReconstructionThesis.pdf
2. Sample Implementation in PCL: https://pcl.readthedocs.io/projects/tutorials/en/latest/greedy_projection.html
