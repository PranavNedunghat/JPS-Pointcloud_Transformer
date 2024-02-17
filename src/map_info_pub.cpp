#include "ros/ros.h"
#include<stdio.h>
#include<cstdlib>
#include<cmath>
#include<vector>
#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include "JPS/mapinfo.h"
#include "JPS/pathpoints.h"
#include<sensor_msgs/PointCloud2.h>
#include<geometry_msgs/Point.h>
#include<nav_msgs/Odometry.h>
#include<Eigen/Geometry>
#include<Eigen/Dense> 
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


//class PointCloudTransformer
//{
//public:
//Eigen::Matrix4d Trans_Matrix;

//void pointcloud_subscriberCallback(const sensor_msgs::PointCloud2& cloud_msg)
//{
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::fromROSMsg(*cloud_msg, *cloud);
 // for (size_t i = 0; i < cloud->size(); i++)
 // {
  // Eigen::Vector4d vec(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z,1.0);
   
   
 // }
//}
//};


int main ()
{
 // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  //cloud->width  = 5;
  //cloud->height = 1;
  //cloud->points.resize (cloud->width * cloud->height);

  //for (std::size_t i = 0; i < cloud->points.size (); ++i)
  //{
  //  cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
  //  cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
   // cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  //}

 // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  //viewer.showCloud (cloud);

  //while (!viewer.wasStopped ())
  //{
  //}

 // return 0;
}
