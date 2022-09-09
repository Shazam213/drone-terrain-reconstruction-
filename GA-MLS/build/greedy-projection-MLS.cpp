#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/mls.h>

int main (int argc, char** argv)

{
    //Load the MLS-obtained-results in a Point Cloud of suitable type.
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::io::loadPCDFile ("CSite1_orig-utm-mls.pcd", *cloud_with_normals);

    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    
    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (30);
    
    // Set typical values for the parameters
    gp3.setMu (3);
    gp3.setMaximumNearestNeighbors (1000);
    gp3.setMaximumSurfaceAngle(M_PI/3); // 60 degrees
    gp3.setMinimumAngle(M_PI/9); // 20 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

     // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);
    
    // Save mesh
    pcl::io::saveVTKFile ("mesh.vtk", triangles);   

    // Finish
    return (0);
}