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

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1)
        {PCL_ERROR("File reading insuccessful. File maybe in a different location, empty or corrupted.");}
    
    std::cout << "Loaded "<< (cloud->width * cloud->height) <<" data points from test_pcd.pcd with the following fields: "<<std::endl;
    for (const auto& point: *cloud)
    {
        std::cout << "X:    " << point.x<< "Y:    "<< point.y<<"Z:    "<< point.z << std::endl;
    }

    for (const auto& point: *cloud)
    {
        int K = 2;

        std::cout << "K nearest neighbor search at point (" << point.x << ", " << point.y << ", " << point.z << ") with K=" << K << std::endl;
        
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

        kdtree.setInputCloud (cloud);

        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if ( kdtree.nearestKSearch (point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            {
                std::cout << "    "  <<   (*cloud)[ pointIdxNKNSearch[i] ].x << " " << (*cloud)[ pointIdxNKNSearch[i] ].y<< " " << (*cloud)[ pointIdxNKNSearch[i] ].z<<std::endl;
                //std::cout<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
            }
        }

        /*pcl::PointCloud<pcl::PointXYZ>::Ptr neighbours (new pcl::PointCloud<pcl::PointXYZ>);
        neighbours->width = pointIdxNKNSearch.size ();
        neighbours->height = 1;
        neighbours->points.resize (neighbours->width * neighbours->height);

        /*const::Eigen::Matrix4f centroid;

        Eigen::Matrix3f covariance_matrix;

        compute3DCentroid (cloud, centroid);*/

        //computeCovarianceMatrix(&cloud, &centroid, covariance_matrix);
        
/*
        for (std::size_t i =1; i <= covariance_matrix.rows(); i++)
        {
            std::cout<<covariance_matrix.row(i)<<std::endl;
        }
        */

        float xm =0, ym=0, zm=0;

        for (std::size_t i =0; i < pointIdxNKNSearch.size(); ++i)
        {
            xm = xm + (*cloud)[ pointIdxNKNSearch[i] ].x;
            ym = ym + (*cloud)[ pointIdxNKNSearch[i] ].y;
            zm = zm + (*cloud)[ pointIdxNKNSearch[i] ].z;
        }

        xm = xm/pointIdxNKNSearch.size (); ym = ym/pointIdxNKNSearch.size (); zm = zm/pointIdxNKNSearch.size ();

        pcl::PointXYZ centroid;

        centroid.x = xm; centroid.y = ym; centroid.z = zm; 

        std::cout<<"Centroid: X: "<<centroid.x<<" Y: "<<centroid.y<<" Z: "<<centroid.z<<std::endl;

        Eigen::Matrix3f covariance_matrix;
        
        Eigen::Vector4f xyz_centroid;

        pcl::compute3DCentroid (cloud, xyz_centroid);


        /*
        
        Eigen::Matrix3f covariance_matrix;
        
        pcl::computeCovarianceMatrix(cloud, centroid, covariance_matrix);
        std::cout<<"Covariance Matrix: "<<std::endl;

        /*for (std::size_t i =1; i <= covariance_matrix.rows(); i++)
        {
            std::cout<<covariance_matrix.row(i)<<std::endl;
        }*/
    }

    return 0;

}