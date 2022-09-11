<h1> Principal Component Analysis & Kd-Trees for Normal Estimation </h1>

<h2> Need for a Pre-Processing Algorithm </h2>

* The majority of the real-world datasets that would be obtained by the LiDAR sensors are highly susceptible to be missing, inconsistent, and noisy due to their **heterogeneous origin**.
* Applying meshing algorithms on this noisy data would therefore not give quality results as they would fail to identify patterns effectively. Data pre-processing is, therefore, important to improve the overall data quality.<br>

>![image](https://user-images.githubusercontent.com/95737452/189468965-daf909c4-e687-42d7-8257-b919b9d6394e.png)

<h2> Features of Principal Component Analysis </h2>

* Principal component analysis (PCA) simplifies the complexity in high-dimensional data while retaining trends and patterns. It does this by transforming the data into fewer dimensions, which act as summaries of features.
* PCA reduces data by geometrically projecting them onto lower dimensions called principal components (PCs), with the goal of finding the best summary of the data using a limited number of PCs.
* The first PC is chosen to minimize the total distance between the data and their projection onto the PC. By minimizing this distance, we also maximize the variance of the projected points, σ^2. The second (and subsequent) PCs are selected similarly, with the additional requirement that they be uncorrelated with all previous PCs.

<h2> Process Flow </h2>

![image](https://user-images.githubusercontent.com/95737452/189506917-7f86e53b-542d-443f-bd3a-b89ec155a869.png) <br><br>

1. Creating a **cloud** to hold our input Point Cloud Data; since the data consists of XYZ coordinates of points, the following choice of cloud is made: <br>
![image](https://user-images.githubusercontent.com/95737452/189470099-2269aafb-016c-438f-a7ff-8245867edaf9.png) <br><br>
   Now, we load our data into this cloud, as follows: <br><br>
![image](https://user-images.githubusercontent.com/95737452/189470187-5e90b155-259a-458e-8bc0-0b0f205fabd4.png)

2. Next, we create a **normal estimation class**, object of which will be used to compute the point normals at each point in the cloud. We then assign the input cloud to it. <br><br>
![image](https://user-images.githubusercontent.com/95737452/189470287-fe52d59f-a33b-458c-8812-04f498ec91cf.png) <br><br>

3. We now create a **Kd-Tree** representation, which will be utilised to search for the k-nearest neighbours of the query point, and set the search method for the normal estimation object to this tree.<br><br>
![image](https://user-images.githubusercontent.com/95737452/189506408-c93a93df-ee39-4c44-9a2b-d8c40565f9de.png)

4. Now, we specify the maximum radius for the function to consider for finding the nearest neighbours, and a placeholder to hold the calculated cloud normals. <br><br>
![image](https://user-images.githubusercontent.com/95737452/189506505-19068c54-7161-43fa-9fd4-2d07a2a05423.png) <br>
![image](https://user-images.githubusercontent.com/95737452/189506961-4aa700de-ba9c-468e-aeb1-c3a719fdb6c9.png)

5. The actual compute call from the NormalEstimation class does nothing internally but the following:<br><br>
 ![image](https://user-images.githubusercontent.com/95737452/189506554-d38c68dd-18ef-4065-9807-e91df2dfe7e7.png)

  For each point p in cloud P: <br>
  * Get the nearest neighbors of point P using the nearest neighbor search using kd-trees (How do Kd-Trees work? Keep scrolling).<br>
  * Compute the mean vertex for the neighborhood and subtract the mean from all the points, to make the data *mean centered* . <br><br>
  ![image](https://user-images.githubusercontent.com/95737452/189506664-5ac74046-912e-45b0-88c0-664f8b3ff9a0.png)
  * Using the mean vertex of the neighborhood, calculate the centroid for the neighborhood.The centroid is used for calculating the covariance matrix. Mathematically, ![image](https://user-images.githubusercontent.com/95737452/189506679-6a1c4e1a-dff0-4bc8-9bd6-167529bfff5d.png) <br><br>
  ![image](https://user-images.githubusercontent.com/95737452/189506726-f63ad6bc-ffab-431b-8382-738685bb239e.png)
  * Calculate the Eigen Vector and the Eigen Values of the Covariance Matrix. <br>
  * A local coordinate system is created by taking a cross product of the eigen vector normal with its unit orthogonal. 
  ![image](https://user-images.githubusercontent.com/95737452/189506803-acee25cd-23c4-4526-9ce4-cb035d3f7517.png) <br><br>
    After the first vector is found, the second vector is found calculating the line of maximum covariance in a plane perpendicular to the fist, and then the third vector, being perpendicular to both, is found as their cross product. <br><br>
    The normalized localU gives us the surface normal. It has been mathematically found to be equivalent to the **eigen vector corresponding to the smallest eigen value** <br><br>
  * Check if n is consistently oriented towards the viewpoint and flip otherwise.<br><br>
 
 



