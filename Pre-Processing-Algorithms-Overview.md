<h1>Pre-Processing Algorithms</h1>


<h2>Important Terminologies</h2>

  * **Modeling** : The mathematical construction and computer implementation of an object, by defining points in a 3D array.
  * **Surface**: A compact, connected, orientable 2D or 3D manifold, possibly with boundary.
  * **Mesh**: A collection of triangular (or quadrilateral) contiguous, non-overlapping faces joined together along their edges.

<h2>Technologies Used </h2>

  * **PCL** : The Point Cloud Library (**PCL**) is a standalone, large scale, open project for 2D/3D image and point cloud processing. It implements a set of algorithms designed to help work with 3-D data, in particular point clouds.<br>
    Following classes of PCL were implemented: **PCA** (Principal Component Analysis), **MLS** (Moving Least Squares)
   
<h2> Algorithm 1: Principal Component Analysis </h2>

  * The input point cloud in the project takes only the position values of the point cloud, so we approximate the normal data directly from the point cloud.
  * Steps involved:<br> For each point p in a Point Cloud P:
      <ol>
      1. Get the nearest neighbors of point P using the nearest neighbor search using kd-trees.<br>
      <ol>
        i. Compute the mean vertex for the neighborhood and subtract the mean from all the points.
        ii.Using the mean vertex of the neighborhood, calculate the centroid for the neighborhood.
        iii.The centroid is used for calculating the covariance matrix.
        iv.Calculate the Eigen Vector and the Eigen Values of the Covariance Matrix.
      </ol>
      2. A local coordinate system is created by taking a cross product of the eigen vector normal with its unit orthogonal.<br>
         LocalV = eigenNormal.unitOrthogonal(); LocalU = eigenNormal.cross(localV); <br>
         The normalized localU gives us the surface normal.<br>
      3. Check if n is consistently oriented towards the viewpoint and flip otherwise.</ol> <br>
  
  <h2> Algorithm 2: Moving Least Squares </h2>

