<h1> Surface Reconstruction: Approach #1 </h1>

# Table of Contents
1. [Process Flow](#Process-Flow)
2. [Primary Input Image](#Primary-Input-Image)
3. [Downsampling with VoxelGrid Filter](#Downsampling-with-VoxelGrid-Filter)
4. [Surface Normal Estimation with Principal Component Analysis](#Surface-Normal-Estimation-with-Principal-Component-Analysis)
5. [Mesh Construction with Greedy Algorithm](#Mesh-Construction-with-Greedy-Algorithm)
6. [Files Mentioned](#Files-Mentioned)
7. [References](#References)

<h2> Process Flow <a name="Process-Flow"></a></h2>

> **Step 1**: Downsample the input data cloud by applying the [**VoxelGrid**](https://pointclouds.org/documentation/classpcl_1_1_voxel_grid_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html) Filter, which consolidates the data points in such a manner so as to include the centroid of all the points within every Leaf of the grid. <br>
> **Step 2:** From the *downsampled cloud*, [estimate the point normals](https://pointclouds.org/documentation/classpcl_1_1_normal_estimation.html) using the *input cloud* to look for nearest neighbours, by implementing **Principal Component Analysis**. <br>
> **Step 3**: From the estimated set of normals, create a mesh to reconstruct the surface, by implementing the **Greedy Algorithm**. <br>

<h2>Primary Input Image<a name="Primary-Input-Image"></a></h2>

![image](https://user-images.githubusercontent.com/95737452/189767203-5c820b68-d15b-4222-961a-0172cb9ee2c1.png)

<h2> Downsampling with VoxelGrid Filter <a name="Downsampling-with-VoxelGrid-Filter"></a></h2>

<h3> Implementation </h3>

> We found the need to implement a downsampling algorithm before pre-processing after observing the results obtained from Principal Component Analysis, which estimated the normals at *every* point in the input point cloud, which defeated the purpose of it being used as a pre-processing algorithm (aka to consolidate the data set). <br>
> The `pcl::VoxelGrid` class creates a 3D voxel grid (think about a voxel grid as a set of tiny 3D boxes in space, whose size is described by the parameter `leaf_size_`) over the input point cloud data. Then, in each voxel, all the points present will be *approximated (i.e., downsampled) with their centroid*.  <br>
> This approach is a bit slower than approximating them with the center of the voxel, but it represents the underlying surface more accurately. <br>

<h3> Output </h3>

![Post-VoxelGrid-Output](https://user-images.githubusercontent.com/95737452/189767346-a6b294f3-d122-408d-9d66-31af497ecd65.png) <br>

<h2> Surface Normal Estimation with Principal Component Analysis <a name="Surface-Normal-Estimation-with-Principal-Component-Analysis"></a></h2>

<h3> Implementation </h3>

> [Principal Component Analysis](https://pointclouds.org/documentation/classpcl_1_1_p_c_a.html) works on reducing the dimensionality of the input dataset. Principal components are extracted by singular values decomposition on the covariance matrix of the centered input cloud. <br>
> The input point cloud in our project takes only the position values of the point cloud, and we approximate the normal data directly from the point cloud. <br>
> Mathematically, for each point in the downsampled cloud, the nearest neighbours of this point in the search cloud are found, using the [`Kd-Trees`](https://pointclouds.org/documentation/classpcl_1_1_kd_tree.html) search data representation. <br> 
> Using these points, a data matrix is created and mean centered, and a covariance matrix is obtained which is used to understand how each point varies in comparison to every other point in the neighbourhood. <br>
> The [eigenvectors and eigenvalues](https://www.youtube.com/watch?v=PFDu9oVAE-g&list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab&index=14) of this matrix are calculated and the eigenvector corresponding to the smallest eigenvalue is the estimated normal.

<h3> Output </h3>

![Post-PCA-Output](https://user-images.githubusercontent.com/95737452/189989562-695e951e-fb66-44a6-9d55-efbe7836a41a.png)

<h2> Mesh Construction with Greedy Algorithm <a name="Mesh-Construction-with-Greedy-Algorithm"></a></h2>

<h3> Implementation </h3>

> The [Greedy Projection](https://pointclouds.org/documentation/classpcl_1_1_greedy_projection_triangulation.html) algorithm works by maintaining a list of points from which the mesh can be grown (“fringe” points) and extending it until all possible points are connected. Triangulation is performed locally, by projecting the local neighborhood of a point along the point’s normal and connecting unconnected points. <br>

<h3> Output </h3>

![Post-Greedy-Output](https://user-images.githubusercontent.com/95737452/189991746-f344cddc-0c55-482e-9eb3-5cf7dae9e631.png)

<h2> Files Mentioned <a name = "Files-Mentioned"></a></h2>

Both files have been added, Commit ID: `11b267f1ef10db36505796716b82646701155f0c`
* The Code: `VGF-PCA-GA.cpp`
* The Sample File: `table-scene.pcd`

<h2> References <a name="References"></a></h2>

* Surface Reconstruction Thesis: https://nccastaff.bmth.ac.uk/jmacey/OldWeb/MastersProjects/MSc13/14/Thesis/SurfaceReconstructionThesis.pdf 
* VoxelGrid Downsampling Understanding: https://pcl.readthedocs.io/en/latest/voxel_grid.html
* Principal Component Analysis & Normal Estimation:
    <ol>
    1. Theory: <br>
        <ol>
        i. https://slideplayer.com/slide/3431756/ <br>
        ii. https://www.diva-portal.org/smash/get/diva2:1358700/FULLTEXT01.pdf Section 2.3, Normal Estimation <br>
        </ol>
    2. Understanding: https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html#estimating-the-normals
    </ol>
* Greedy Algorithm & Mesh Reconstruction:
    1. Understanding: https://pcl.readthedocs.io/projects/tutorials/en/latest/greedy_projection.html
    2. Official Implementation in PCL: https://ias.informatik.tu-muenchen.de/_media/spezial/bib/marton09icra.pdf
