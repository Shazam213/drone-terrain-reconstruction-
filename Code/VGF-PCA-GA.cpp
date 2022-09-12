#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/conversions.h>


/* Algorithm: 
    Step 1: Downsample the data by applying the VoxelGrid Filter.
    Step 2: From the downsampled cloud, estimate the point normals using the input cloud to look for nearest neighbours, by implementing Principal Component Analysis.
    Steep 3: From the estimated normals, create a mesh to reconstruct the surface, by implementing the Greedy Algorithm. 
*/

int main()
{

    /* Input */

    //'search_cloud' of PointCloud<PointXYZ> type stores input data cloud that will undergo downsampling andbe used for getting the nearest neighbours.
    pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("table-scene.pcd", *search_cloud);

    //Displaying the number of points in input cloud before downsampling.
    std::cerr << "PointCloud before filtering consists of: " << (search_cloud->width * search_cloud->height) << " data points." << std::endl;

    
    
    /* Illustration of downsampling of data. */

    //Object created for filtering data.
    pcl::VoxelGrid<pcl::PointXYZ> sor;

    //Assigning the cloud to be filtered through to the filtering object.
    sor.setInputCloud (search_cloud);

    //LeafSize = Describes the size of the cells which shall accumulate points of cloud.
    sor.setLeafSize (0.01f, 0.01f, 0.01f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_blob (new pcl::PointCloud<pcl::PointXYZ>);

    //When this call is made, the filter() function references the applyFilter() function which then actually implements downsampling by computing the centroid of the points in each grid square of the size 'LeafSize'. The placeholder cloud_filtered is utilised to store the downsampled cloud.
    sor.filter (*filter_blob);

    //Displaying points in cloud after downsampling.
    std::cerr << "PointCloud after filtering consists of : " << (filter_blob->width * filter_blob->height) << " data points." << std::endl;

    //Saving the downsampled cloud to a PCD file.
    pcl::io::savePCDFile ("table-scene-downsampled.pcd", *filter_blob);

    
    
    /* Illustration of estimation of normals */

    //Creating the object to be used for normal estimation.
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

    //Defining the input cloud for calculation of surface normals, which is the downsampled cloud 'cloud_filtered'.
    ne.setInputCloud (filter_blob);

    //Defining the search surface cloud to be analysed for nearest neighbours, which is 'cloud'.
    ne.setSearchSurface (search_cloud);

    //Initialising the representaion that will be utilised to look for nearest neighbours, Kd-Tree.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    
    //Defining the search method for obtaining nearest neighbours, and passing it to the normal estimation object. 
    ne.setSearchMethod (tree);

    // Placeholder for containing the estimated normals.
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
      
    // Defining the maximum search radius for the nearest neighbors.
    ne.setRadiusSearch (0.03);

    /* When this call is made, the compute() function references the computePointNormal() function which references two other functions: computeMeanAndCovarianceMatrix() to calculate the Covariance Matrix and solvePlaneParameters() to get the smallest eigenvalue and the eigenvector corresponding to it, based on the Covariance Matrix previously obtained. 
    
    Mathematically, for each point in the downsampled cloud, the nearest neighbours of this point in the search cloud are found. Using these points, a data matrix is created and mean centered, and a covariance matrix is obtained which is used to understand how each point varies in comparison to every other point in the neighbourhood. The eigenvectors and eigenvalues of this matrix are calculated and the eigenvector corresponding to the smallest eigenvalue is the estimated normal.*/

    ne.compute (*cloud_normals);

    std::cerr << "PointCloud after normal estimation consists of : " << (cloud_normals->width * cloud_normals->height) << " data points." << std::endl;

    //Saving the calculated normals to a PCD file.
    pcl::io::savePCDFile ("table-scene-downsampled-pca.pcd", *cloud_normals);


    /* Illustration of Mesh Reconstruction */

    //Defining a placeholder to contain the concatenation of points from the downsampled cloud and the estimated normals at these points.
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*filter_blob, *cloud_normals, *cloud_with_normals);

    // Creating another Kd-Tree for obtaining nearest neighbours.
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.025);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    //Saving the constructed mesh to a VTK file.
    pcl::io::saveVTKFile ("mesh.vtk", triangles);  


    return 0;
}