#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/pca.h>
#include <vector>
#include <ctime>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>
#include <pcl/features/normal_3d.h>

/*

1. Get the nearest neighbors of point P using the nearest neighbor search using kd-tree discussed
in chapter 9.
i.Compute the mean vertex for the neighborhood and subtract the mean from all the points.
ii.Using the mean vertex of the neighborhood, calculate the centroid for the neighborhood.
iii.The centroid is used for calculating the covariance matrix.
iv.Calculate the Eigen Vector and the Eigen Values of the Covariance Matrix.
2. A local coordinate system is created by taking a cross product of the eigen vector normal with its
unit orthogonal.
LocalV = eigenNormal.unitOrthogonal();
LocalU = eigenNormal.cross(localV);
The normalized localU gives us the surface normal.
3. Check if n is consistently oriented towards the viewpoint and flip otherwise

*/

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("bun0.pcd", *cloud) == -1)
        {PCL_ERROR("File reading insuccessful. File maybe in a different location, empty or corrupted.");}
    
    std::cout << "Loaded "<< (cloud->width * cloud->height) <<" data points from bun0.pcd with the following fields: "<<std::endl;
    for (const auto& point: *cloud)
    {
        std::cout << "X:    " << point.x<< "Y:    "<< point.y<<"Z:    "<< point.z << std::endl;
    }

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

    ne.setInputCloud (cloud);

    ne.setRadiusSearch (0.03);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    
    ne.setSearchMethod (tree);

    ne.compute (*cloud_normals);

    std::cout<<"Normals of the point cloud data: "<<std::endl;

     for (const auto& point: *cloud_normals)
    {
        std::cout << point<< std::endl;
    }

    return 0;

}