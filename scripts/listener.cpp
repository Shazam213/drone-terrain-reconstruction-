#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <cmath>
#include <thread>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <boost/foreach.hpp>
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/surface/mls.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <cmath>

typedef pcl::PointCloud <pcl::PointXYZ> PointCloud;
using namespace std;
using namespace sensor_msgs;


//'cloud_append' of PointCloud<PointXYZ> type stores the entire collected Point Cloud Data of the terrain.
pcl::PointCloud<pcl::PointXYZ> cloud_append ; 

void callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  /* Input from ROS Topic "/3d_image/3d_cloud/" */

  //'cloud' Ptr of PointCloud<PointXYZ> type stores input data cloud of every callback that will undergo downsampling andbe used for getting the nearest neighbours.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //'cloud_input' Ptr of PointCloud<PointXYZ> type stores the downsampled data cloud of every callback.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_new (new pcl::PointCloud<pcl::PointXYZ>);
  
  

  //Object created for filtering data for this callback.
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  std::cerr <<"Loop starting "<< std::endl;
  //Obtaining the Point Cloud Data from ROS for this callback.
  pcl::fromROSMsg (*msg, *cloud);

  
  /* The coordinates obtained from the sensors do not have a fixed frame of reference, since they are defined considering the drone itself as origin. Hence, the following few steps are used to obtain the coordinates of the drone with respect to the map origin, and hence the poind cloud data can be defined wrt map origin using translation. */

  /* Input from ROS Service "/gazebo/get_model_state" */

  ros::NodeHandle nhs;
  // Creating a service client that utilises the "/gazebo/get_model_state" service here to get the XYZ coordinates (Pose) of the drone wrt map origin in this callback.
  ros::ServiceClient client = nhs.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  //Setting the required getmodelstate parameters in this callback.
  gazebo_msgs::GetModelState getmodelstate;
  getmodelstate.request.model_name="sjtu_drone";
  //Stores the Pose in this callback
  geometry_msgs::Pose pose;
  // geometry_msgs::Quaternion q;
  geometry_msgs::Twist twist;
  double yaw=0.0;
  double roll=0.0;
  //Get the Pose and the yaw angle of the drone
  if (client.call(getmodelstate))
  {
    std::cerr <<"Service called. "<<std::endl;
    
    pose.position.x = getmodelstate.response.pose.position.x;
    pose.position.y = getmodelstate.response.pose.position.y;
    pose.position.z = getmodelstate.response.pose.position.z;
    pose.orientation.x=getmodelstate.response.pose.orientation.x;
    pose.orientation.y=getmodelstate.response.pose.orientation.y;
    pose.orientation.z=getmodelstate.response.pose.orientation.z;
    pose.orientation.w=getmodelstate.response.pose.orientation.w;

    double siny_cosp = 2 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y);
    double cosy_cosp = 1 - 2 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
  
  }
  else
  {
    ROS_INFO("FAILED");
  }

  /*Downsampling the data before translating it in order to save computational time in this callback. */

  //Assigning the cloud to be filtered through to the filtering object in this callback.
  sor.setInputCloud (cloud);
  //LeafSize = Describes the size of the cells which shall accumulate points of cloud in this callback.
  sor.setLeafSize (0.04f, 0.04f, 0.04f);
  //When this call is made, the filter() function references the applyFilter() function which then actually implements downsampling by computing the centroid of the points in each grid square of the size 'LeafSize'. The placeholder cloud_filtered is utilised to store the downsampled cloud.
  sor.filter (*cloud_input);
  sor.filter (*cloud_input_new);
 
  // Transformation of the frames

  for( size_t i = 0; i < cloud_input->size()-1; i++)
  {
    
    cloud_input->points[i].x = -((cloud_input_new->points[i].x+ pose.position.x)*cos(yaw)-(cloud_input_new->points[i].y- pose.position.y)*sin(yaw)) ;
    cloud_input->points[i].y = ((cloud_input_new->points[i].y- pose.position.y)*cos(yaw)+(cloud_input_new->points[i].x+ pose.position.x)*sin(yaw)) ;
    cloud_input->points[i].z = -cloud_input_new->points[i].z + pose.position.z;
    // double Xd=pose.position.x;
    // double Yd=pose.position.y;
    // double Xl=cloud_input_new->points[i].x;
    // double Yl=cloud_input_new->points[i].y;

    // cloud_input->points[i].x=Xl*sin(yaw)*tan(yaw)-Yl*sin(yaw)+Xd-Xl*cos(yaw);
    // cloud_input->points[i].y=Yd-Xl*sin(yaw)+Yl*cos(yaw);
    cloud_append.push_back(cloud_input->points[i]);
  }

  
    std::cerr<<"Delay starting"<<endl;
    ros::Duration(35).sleep();
    std::cerr<<"Delay ended"<<endl;
}


int main(int argc, char** argv)
{

  /* Following lines of code serve to obtain Point Cloud Input from the sensors */
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  std::cerr <<"Listener starting..."<< std::endl;

  // subscribing to the topic to which PointCloud2 data from the sensors is being published.  
  ros::Subscriber sub = nh.subscribe ("/3d_image/3d_cloud/", 1, callback);
  //Re-iteration.
  ros::spin();
  
  //Storing the source PCD file obtained upon mapping the terrain completely.
  pcl::io::savePCDFile("source-cloud.pcd",cloud_append);
  std::cerr << "Data file created."<< std::endl;

/* Estimation of surface normals with Moving Least Squares */

//'search_cloud' Ptr of PointCloud<PointXYZ> type stores input data cloud on which normals will be computed
pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile ("source-cloud.pcd", *search_cloud);
pcl::VoxelGrid<pcl::PointXYZ> sor1;
 sor1.setInputCloud (search_cloud);

    //LeafSize = Describes the size of the cells which shall accumulate points of cloud.
    sor1.setLeafSize (0.08f, 0.08f, 0.08f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_blob (new pcl::PointCloud<pcl::PointXYZ>);

    //When this call is made, the filter() function references the applyFilter() function which then actually implements downsampling by computing the centroid of the points in each grid square of the size 'LeafSize'. The placeholder cloud_filtered is utilised to store the downsampled cloud.
    sor1.filter (*filter_blob);
std::cerr<<"Downsampling done"<<std::endl;
//MLS object for implementation of Moving Least Squares algorithm.
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//Concatenated cloud that stores both the XYZ Coordinates of a point and the Normal at that point.
  pcl::PointCloud<pcl::PointNormal> mls_points;
//Initialising the representaion that will be utilised to look for nearest neighbours, Kd-Tree.
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//When set to true, computed normals along with being computed, are also saved.
  mls.setComputeNormals (true);
//Defining the input cloud for calculation of surface normals
  mls.setInputCloud (filter_blob);
//Setting the required MLS Parameters
  mls.setPolynomialOrder (3);
//Defining the search method for obtaining nearest neighbours, and passing it to the normal estimation object.
  mls.setSearchMethod (tree);
//Defining the maximum search radius for the nearest neighbors.
  mls.setSearchRadius (0.5);
//Processing Normals and saving it to the provided placeholder.
  mls.process (mls_points);
  pcl::io::savePCDFile ("output_mls.pcd", mls_points);   
  std::cerr<<"MLS completed successfully"<< std::endl;

/* Meshing Algorithm */

  //'cloud_with_normals' Ptr of PointNormal type that stores the XYZ Coordinates and Normal of a particular point as computed previously.
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::io::loadPCDFile ("output_mls.pcd", *cloud_with_normals);
  //Initialising the representaion that will be utilised to look for nearest neighbours, Kd-Tree.
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  //Defining the input cloud for meshing.
  tree2->setInputCloud (cloud_with_normals);

  //GP object for implementation of Greedy Projection Triangulation algorithm.
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  //Placeholder to save the triangular mesh.
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.9);

  // Set typical values for the parameters
  gp3.setMu (3);
  gp3.setMaximumNearestNeighbors (1500);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(true);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);
  pcl::io::saveVTKFile ("mesh1.vtk", triangles);
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();
  std::cerr<<"Greedy triangultation completed successfully"<< std::endl;
  return 0;
}



