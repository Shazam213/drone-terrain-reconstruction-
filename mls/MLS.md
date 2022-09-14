# Moving Least Squares
## What Is the Least Squares Method? 
* The least squares method is a statistical procedure to find the best fit for a set of data points by minimizing the sum of the offsets or residuals of points from the plotted curve.
* The least-squares explain that the curve that best fits is represented by the property that the sum of squares of all the deviations from given values must be minimum, i.e:
  ![image1](../assets/Screenshot%20from%202022-09-15%2003-00-01.png)
  ![image2](../assets/Screenshot%20from%202022-09-15%2003-01-10.png) 
* Moving least square method is when the data is fed continously and simulatneously we get output of all the data present by applying the least squares method.
  
## Moving Least Squares on PCD
* Moving Least Squares (MLS) surface reconstruction method can be used to smooth and resample noisy data.
* Some of the data irregularities (caused by small distance measurement errors) are very hard to remove using statistical analysis. To create complete models, glossy surfaces as well as occlusions in the data must be accounted for. In situations where additional scans are impossible to acquire, a solution is to use a resampling algorithm, which attempts to recreate the missing parts of the surface by higher order polynomial interpolations between the surrounding data points. By performing resampling, these small errors can be corrected and the “double walls” artifacts resulted from registering multiple scans together can be smoothed.
  ![image3](../assets/Screenshot%20from%202022-09-15%2003-04-13.png)
* On the left side we see the noisy data which has been smoothened using the moving least square algorithms.
* Moving least squares first computes the nornmals and then smoothens the curve to remove noises from the data set.
    
        mls.setComputeNormals (true);
    * Setting the bool value to true computes the normals and saves the computed normal data to the pointer.

* Parameters of the mls function:
  
        mls.setInputCloud (cloud);
        mls.setPolynomialOrder (2);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.03); 

    * mls.setInputCloud gives the input in form of pointer
    * mls.setPolynomialOrder() sets the order of polynomial that we want to fit using the least square method. this can be skipped to speed up smoothning of the curve.
    * mls.setSearchMethod (tree) sets the method to search for the nearby points for the smoothning, we use Kd trees and nearest neighbour method to find the adjacent points.
    * mls.setSearchRadius(), sets the radius to search for the points using Kd trees.
    
        
            mls.process();
    * This function is used to finally reconstruct the entire data set after smoothning and removing all the noises. this function first computes the normals and then applies moving least squares method for smoothning. The normals are calculated by using the Eigen values and Eigen vectors of the points in the point cloud data as used in the PCA method.


## Results and Output.
* we tried using MLS on a point cloud dataset of a topographic surface.
* The input data:
  * ![image4](../assets/Screenshot%20from%202022-09-15%2003-29-48.png)
* Output of MLS when the search radius was set to 0.03:
  * ![image5](../assets/Screenshot%20from%202022-09-15%2003-29-56.png)
* Output of MLS when the search radius was set to 30:
  * ![image6](../assets/Screenshot%20from%202022-09-15%2003-30-02.png)
* Thus, we observed that the search radius plays an important role as changing the search radius drastically changes the output. Here the search radius is the radius input in the mls.setSearchRadius() function and it is the radius of the nearest neighbourhood. 
