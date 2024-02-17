  #include<iostream>
#include<stdio.h>
#include<cstdlib>
#include<cmath>
#include<vector>
#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include<sensor_msgs/PointCloud2.h>
#include<geometry_msgs/Point.h>
#include<nav_msgs/Odometry.h>
#include<Eigen/Geometry>
#include<Eigen/Dense> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include<pcl_ros/point_cloud.h>
#include<JPS/path_planning/JumpPointSearch.h>
#include<visualization_msgs/MarkerArray.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/PoseStamped.h>
#include<topic_tools/shape_shifter.h>
#include "JPS/jps_service_message.h"
using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
static int counter = 0;
class JPSNodelet
{
public:
Eigen::Matrix4d Trans_Matrix;
Eigen::Matrix4d inv_Trans_Matrix;
double x_dest;
double y_dest;
double z_dest;
int map_size;
int offset;
int offset_z;
double cell_size;
int max_iterations;
int pad_size;
double threshold;
int waypoint_num;
bool is_dest_set;
bool is_waypoint_set;
float drone_velocity;
float drone_vel_threshold;
Eigen::Vector3d drone_vel;
nav_msgs::Path waypoints;
boost::shared_ptr<const nav_msgs::Odometry_<std::allocator<void> > > odom_msg;
boost::shared_ptr<const topic_tools::ShapeShifter> map_msg;
void pointcloud_subscriberCallback(const topic_tools::ShapeShifter::ConstPtr& msg);
void pose_callback(const nav_msgs::Odometry::ConstPtr& odom);
void pose_publisher70Hz(const ros::TimerEvent& event); 
void find_free_node(int ***ogm,int &x, int &y, int &z, int max_scope);
void JPS_Publisher(double x_dest, double y_dest, double z_dest);
void Visualiser(vector <struct path_planning::JumpPointSearch::Node> rpath);
bool service_callback(JPS::jps_service_message::Request &req, JPS::jps_service_message::Response &res);
void waypoint_callback(const nav_msgs::Path::ConstPtr& wp);
bool check_waypoint(double x, double y, double z);
path_planning::JumpPointSearch jps;
ros::NodeHandle n;
ros::Publisher marker_pub;
ros::Publisher waypoint_pub;
ros::Publisher pose_pub_70_Hz;
ros::Subscriber pc_sub;
ros::Subscriber pose_sub;
ros::ServiceServer service;
ros::Subscriber waypoint_sub;
ros::Timer timer;
nav_msgs::Path goal_waypoints;

JPSNodelet()
{//"/race2/odom_transform_nodelet/odomBinB0_from_transform"
 //"/race5/odom_transform_nodelet/odomBinB0_from_transform"
 odom_msg = NULL;
 map_msg = NULL;
 is_dest_set = false;
 is_waypoint_set = false;
 n = ros::NodeHandle("/quadrotor/jps_server");
 marker_pub = n.advertise<visualization_msgs::Marker>("/JPS_Path", 1);
 waypoint_pub = n.advertise<nav_msgs::Path>("/quadrotor/waypoints",1);
 pose_sub = n.subscribe("/quadrotor/odom",1,&JPSNodelet::pose_callback,this);
 pc_sub = n.subscribe("/voxblox_node/occupied_nodes",1,&JPSNodelet::pointcloud_subscriberCallback,this);
 service = n.advertiseService("/JPS_Server_single_waypoint",&JPSNodelet::service_callback,this);
 waypoint_sub = n.subscribe("/race/waypoints",1,&JPSNodelet::waypoint_callback,this);
 pose_pub_70_Hz = n.advertise<nav_msgs::Odometry>("/race/odom",1);
 n.param("map_size",map_size,20);
 n.param("cell_size",cell_size,0.3);
 n.param("max_iterations",max_iterations,500);
 n.param("pad_size",pad_size,0);
 n.param("threshold",threshold,0.5);
 n.param("waypoint_num",waypoint_num,1);
 timer = n.createTimer(ros::Duration(1.0/10.0), &JPSNodelet::pose_publisher70Hz,this);
 Trans_Matrix.setZero();
 inv_Trans_Matrix.setZero();
 offset_z = offset = int(map_size/2);
 waypoints.header.frame_id = "world";
 jps.set_mapsize(map_size);
 jps.set_max_iter(max_iterations);
 drone_vel_threshold = 0.5;
}

};
void JPSNodelet::pose_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
 odom_msg = odom;
 Eigen::Quaterniond q(odom->pose.pose.orientation.x,odom->pose.pose.orientation.y,odom->pose.pose.orientation.z,odom->pose.pose.orientation.w);
 q.normalize();
 Eigen::Matrix3d Rot_matrix;
 Rot_matrix = q.toRotationMatrix();
 offset_z = int(round(odom->pose.pose.position.z/cell_size));
 if (offset_z > offset)
 {offset_z = offset;}
 Trans_Matrix << Rot_matrix(0,0),Rot_matrix(0,1),Rot_matrix(0,2),odom->pose.pose.position.x,
 		  Rot_matrix(1,0),Rot_matrix(1,1),Rot_matrix(1,2),odom->pose.pose.position.y,
 		  Rot_matrix(2,0),Rot_matrix(2,1),Rot_matrix(2,2),odom->pose.pose.position.z,
 		  0.0, 0.0, 0.0, 1.0;
 inv_Trans_Matrix = Trans_Matrix.inverse();
 drone_vel = Eigen::Vector3d(odom->twist.twist.linear.x,odom->twist.twist.linear.y,odom->twist.twist.linear.z); 
 drone_velocity = sqrt(pow(drone_vel(0),2) + pow(drone_vel(1),2) + pow(drone_vel(2),2));
}

bool JPSNodelet::check_waypoint(double x, double y, double z)
{
 Eigen::Vector4d rel_(x,y,z,1.0);
 rel_ = inv_Trans_Matrix*rel_;
 int rel_x = int(round((rel_(0)/cell_size) + offset));
 int rel_y = int(round((rel_(1)/cell_size) + offset));
 int rel_z = int(round((rel_(2)/cell_size) + offset_z));
 if ((rel_x>=0 && rel_x<=map_size-1)&&(rel_y>=0 && rel_y<=map_size-1)&&(rel_z>=0 && rel_z<=map_size-1))
{
	 if (jps.ogm[rel_x][rel_y][rel_z] == 1)
		{
		 return true;
		}
	 else
		{
		 return false;
		}
}
 else
	{
	 return true;
	}
}

void JPSNodelet::find_free_node(int ***ogm,int &x, int &y, int &z, int max_scope)
{
 if (ogm[x][y][z] == 1)
 {return;
 }
 int scope = 1;
 while (scope < max_scope)
 {
	 for (int i = -scope; i <= scope; i = i + 2*scope)
	 {
		 for (int j = -scope+1; j <= scope-1; j++)
		 {
			 for (int k = -scope+1; k <= scope-1; k++)
			 {
			  if (((x + i)>=0) && ((x + i)<=map_size-1) && ((y + j)>=0) && ((y + j)<=map_size-1) && ((z + k)>=0) && ((z + k)<=map_size-1))
			  {
			  	if (ogm[x+i][y+j][z+k] == 1)
			  	{
			  	 x += i;
			  	 y += j;
			  	 z += k;
			  	 return;
			  	 
			  	}
        }
       }
     }
   }
   for (int k = -scope; k <= scope; k++)
	 {
		 for (int j = -scope; j <= scope; j++)
		 {
       if (((z-scope)>=0) && ((y + j)>=0) && ((y + j)<=map_size-1) && ((x + k)>=0) && ((x + k)<=map_size-1))
       {
        if(ogm[x+k][y+j][z-scope] == 1)
        {
           x += k;
			  	 y += j;
			  	 z -= scope;
			  	 return;
        }
       }
       else if (((z + scope)<=map_size-1) && ((y + j)>=0) && ((y + j)<=map_size-1) && ((x + k)>=0) && ((x + k)<=map_size-1))
       {
        if(ogm[x+k][y+j][z+scope] == 1)
        {
           x += k;
			  	 y += j;
			  	 z += scope;
			  	 return;
        }
       }
       if (((x + k)>=0) && ((x + k)<=map_size-1) && ((y + scope)<=map_size-1) && ((z + j)>=0) && ((z + j)<=map_size-1))
       {
         if(ogm[x+k][y+scope][z+j] == 1)
        {
           x += k;
			  	 y += scope;
			  	 z += j;
			  	 return;
        }
       }
       if (((x + k)>=0) && ((x + k)<=map_size-1) && ((y - scope)>=0) && ((z + j)>=0) && ((z + j)<=map_size-1))
       {
        if(ogm[x+k][y-scope][z+j] == 1)
        {
           x += k;
			  	 y -= scope;
			  	 z += j;
			  	 return;
        }
		   }
		 }
	 }
	 scope += 1;
 }
 ROS_INFO("Unable to find a nearby empty node!!!\n");
 is_dest_set = false;
 return;
}

void JPSNodelet::Visualiser(vector <struct path_planning::JumpPointSearch::Node> rpath)
{
    //ROS_INFO("In Visualizer\n");
    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = "world";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "Jump_Point_Search";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;


    points.id = 0;
    line_strip.id = 1;
   

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    

    points.scale.x = 0.2;
    points.scale.y = 0.2;

    line_strip.scale.x = 0.1;

    points.color.g = 1.0;
    points.color.a = 1.0;
    

    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    
    if(drone_vel(0) <= drone_vel_threshold && drone_vel(1) <= drone_vel_threshold)
    {
	    waypoints.poses.clear();
	    waypoints.header.stamp = ros::Time::now();
	    int k, i;
	  for(k = rpath.size()-2,i = 0;k>=0; i++, k--)
		 {geometry_msgs::Point p;
			Eigen::Vector4d vec_Drone_Frame((rpath[k].current_node_x - offset)*cell_size, (rpath[k].current_node_y - offset)*cell_size,(rpath[k].current_node_z - offset_z)*cell_size,1.0);
	    Eigen::Vector4d vec_World_Frame = Trans_Matrix*vec_Drone_Frame; 
			p.x = vec_World_Frame(0);
			p.y = vec_World_Frame(1);
			p.z = vec_World_Frame(2);
	    if(i<waypoint_num)
	    {
	    		geometry_msgs::PoseStamped w;
	    		w.header.frame_id = "world";
			w.header.stamp = ros::Time::now();
		    	w.pose.position.x = vec_World_Frame(0);
			w.pose.position.y = vec_World_Frame(1);
			w.pose.position.z = vec_World_Frame(2); 
			w.pose.orientation.x = 0.0;
			w.pose.orientation.y = 0.0;
			w.pose.orientation.z = 0.0;
			w.pose.orientation.w = 1.0;
	    		waypoints.poses.push_back(w);
	    }
			points.points.push_back(p);
			line_strip.points.push_back(p);
	   }
		 marker_pub.publish(points);
		 marker_pub.publish(line_strip);
		 waypoint_pub.publish(waypoints);

		 ROS_INFO("Publishing..\n");
    }
}


void JPSNodelet::JPS_Publisher(double x_dest, double y_dest, double z_dest)
{
 //ROS_INFO("In publisher\n");
 Eigen::Vector4d dest(x_dest,y_dest,z_dest,1);
 Eigen::Vector4d dest_rel = inv_Trans_Matrix*dest;
 int dest_rel_x = int(round((dest_rel(0)/cell_size)+offset));
 int dest_rel_y = int(round((dest_rel(1)/cell_size)+offset));
 int dest_rel_z = int(round((dest_rel(2)/cell_size)+offset_z));
 int start_x = offset;
 int start_y = offset;
 int start_z = offset_z;
 if (dest_rel_x<0)
 {dest_rel_x = 0;}
 else if (dest_rel_x>(map_size-1))
 {dest_rel_x = (map_size-1);}
 
 if (dest_rel_y<0)
 {dest_rel_y = 0;}
 else if (dest_rel_y>(map_size-1))
 {dest_rel_y = (map_size-1);}
 
 if (dest_rel_z<0)
 {dest_rel_z = 0;}
 else if (dest_rel_z>(map_size-1))
 {dest_rel_z = (map_size-1);}
 if (jps.ogm[dest_rel_x][dest_rel_y][dest_rel_z] == 0)
 {
  //Eigen::Vector4d d((dest_rel_x-offset)*cell_size, (dest_rel_y-offset)*cell_size,(dest_rel_z-offset_z)*cell_size,1.0);
  //Ei  ROS_INFO("Acquiring Path.....");gen::Vector4d de = Trans_Matrix*d;
  //ROS_INFO("Destination Node is an occupied node!! Finding nearby free node\n");
  //ROS_INFO("Before: %f, %f, %f \n",de(0), de(1),de(2));
  find_free_node(jps.ogm,dest_rel_x,dest_rel_y,dest_rel_z,map_size);
  Eigen::Vector4d da((dest_rel_x-offset)*cell_size, (dest_rel_y-offset)*cell_size,(dest_rel_z-offset_z)*cell_size,1.0);
  Eigen::Vector4d des = Trans_Matrix*da;
  ROS_INFO("After: %f, %f, %f \n",des(0), des(1),des(2));

 }
 if (jps.ogm[start_x][start_y][start_z] == 0)
 {
  find_free_node(jps.ogm,start_x,start_y,start_z,pad_size+3);
  //ROS_INFO("Current position is considered as an occupied node! Finding closest unoccupied node position to consider as Start node.\n");
 }
 if (sqrt(pow((start_x - dest_rel_x),2) + pow((start_y - dest_rel_y),2) + pow((start_z - dest_rel_z),2))*cell_size < threshold)
 {
  //ROS_INFO("Destination is within threshold. Publishing stopped");
  is_dest_set = false;
  return;
 }
  int start[3] = {start_x,start_y,start_z};
  int target[3] = {dest_rel_x,dest_rel_y,dest_rel_z};
  Eigen::Vector4d da0((start_x-offset)*cell_size, (start_y-offset)*cell_size,(start_z-offset_z)*cell_size,1.0);
  Eigen::Vector4d des0 = Trans_Matrix*da0;
  ROS_INFO("Start is: %f, %f, %f \n",des0(0), des0(1),des0(2));
  Eigen::Vector4d da1((dest_rel_x-offset)*cell_size, (dest_rel_y-offset)*cell_size,(dest_rel_z-offset_z)*cell_size,1.0);
  Eigen::Vector4d des1 = Trans_Matrix*da1;
  ROS_INFO("Dest is: %f, %f, %f \n",des1(0), des1(1),des1(2));
  jps.Jump_Point_Search(start,target);
  Visualiser(jps.rpath);
}
 

void JPSNodelet::pointcloud_subscriberCallback(const topic_tools::ShapeShifter::ConstPtr& msg)
{ 

  if (!odom_msg)
  {
   return;
  }

  if (!msg) {
  ROS_INFO("Did not receive pointcloud message!\n");
   return;
  }
  //ROS_INFO("In pointcloud_subscriber beginning\n");
  map_msg = msg;
  if (!(is_dest_set || is_waypoint_set))
  {return;}
  
  if(is_waypoint_set && !goal_waypoints.poses.empty() && !is_dest_set)
  {
    geometry_msgs::PoseStamped waypoint = goal_waypoints.poses.front();
    goal_waypoints.poses.erase(goal_waypoints.poses.begin());
    x_dest = double(waypoint.pose.position.x);
    y_dest = double(waypoint.pose.position.y);
    z_dest = double(waypoint.pose.position.z);
    ROS_INFO("Now moving to waypoint x: %f, y: %f, z: %f\n",x_dest,y_dest,z_dest);
    is_dest_set = true;
  }
  if (sqrt(pow((Trans_Matrix(0,3) - x_dest),2) + pow((Trans_Matrix(1,3) - y_dest),2) + pow((Trans_Matrix(2,3) - z_dest),2)) < threshold)
 {
  is_dest_set = false;
  if(goal_waypoints.poses.empty())
  {
    ROS_INFO("Final destination is within threshold. Path planning stopped");
    is_waypoint_set = false;
    return;
  }
 }
 counter++;
 if(counter%5 != 1 ||!is_dest_set)
 {return;}
  jps.clear_ogm();
  if (msg->getDataType() == "visualization_msgs/MarkerArray")
  {
    auto marker_msg = msg->instantiate<visualization_msgs::MarkerArray>();
   for (const auto& marker : marker_msg->markers[0].points)
    {
      Eigen::Vector4d vec(marker.x,marker.y,marker.z,1.0);
      Eigen::Vector4d inv_vec = inv_Trans_Matrix*vec;
      inv_vec(0) = inv_vec(0)/cell_size + offset;
      inv_vec(1) = inv_vec(1)/cell_size + offset;
      inv_vec(2) = inv_vec(2)/cell_size + offset_z;
      if ((round(inv_vec(0))>=0 && round(inv_vec(0))<=map_size-1)&&(round(inv_vec(1))>=0 && round(inv_vec(1))<=map_size-1)&&(round(inv_vec(2))>=0 && round(inv_vec(2))<=map_size-1))
      {
        jps.padding(pad_size,int(round(inv_vec(0))),int(round(inv_vec(1))),int(round(inv_vec(2))));
      }
    }
  }

  else if (msg->getDataType() == "sensor_msgs/PointCloud2")
  {
    auto cloud_msg = msg->instantiate<PointCloud>();
    for (int i = 0; i < cloud_msg->points.size(); i++)
  {
   Eigen::Vector4d vec(cloud_msg->points[i].x,cloud_msg->points[i].y,cloud_msg->points[i].z,1.0);
   Eigen::Vector4d inv_vec = inv_Trans_Matrix*vec;
   int inv_vec_x = int(round(inv_vec(0)))/cell_size + offset;
   int inv_vec_y = int(round(inv_vec(1)))/cell_size + offset;
   int inv_vec_z = int(round(inv_vec(2)))/cell_size + offset_z;
   if ((inv_vec_x>=0 && inv_vec_x<=map_size-1)&&(inv_vec_y>=0 && inv_vec_y<=map_size-1)&&(inv_vec_z>=0 && inv_vec_z<=map_size-1))
   {
    jps.padding(pad_size,inv_vec_x,inv_vec_y,inv_vec_z);
   }
  }
  }

  else 
  {
    ROS_INFO("Callback message is not of type visualization_msgs::MarkerArray or sensor_msgs::PointCloud2. Unable to construct occupancy grid map!\n");
    return;
  }
  if(!check_waypoint(x_dest, y_dest,z_dest))
  {
    is_dest_set = false;
    return;
  }
  JPS_Publisher(x_dest,y_dest,z_dest);
}

bool JPSNodelet::service_callback(JPS::jps_service_message::Request &req, JPS::jps_service_message::Response &res)
{
  if(!odom_msg)
  {
    ROS_INFO("Did not receive Odometry information!!\n");
    return false;
  }
  if(!map_msg)
  {
    ROS_INFO("Did not receive Pointcloud information!!\n");
    return false;
  }
  if(!req.start)
  {
    is_dest_set = false;
    is_waypoint_set = false;
    res.status = "Stopping Path planning. Clearing waypoint queue.\n";
    goal_waypoints.poses.clear();
    return true;
  }
  x_dest = double(req.x);
  y_dest = double(req.y);
  z_dest = double(req.z);
  is_dest_set = true;
  is_waypoint_set = false;
  odom_msg = nullptr;
  map_msg = nullptr;

    res.status = "Received new destination coordinates!\n";
    return true;
}

void JPSNodelet::waypoint_callback(const nav_msgs::Path::ConstPtr& wp)
{
  if(!odom_msg)
  {
    ROS_INFO("Did not receive Odometry information!!Ignoring waypoints!!\n");
    return;
  }
  if(!map_msg)
  {
    ROS_INFO("Did not receive Pointcloud information!!Ignoring waypoints!!\n");
    return;
  }
  goal_waypoints = *wp;
  is_waypoint_set = true;
  is_dest_set = false;
  ROS_INFO("Received new waypoints!\n");
  return;

}
void JPSNodelet::pose_publisher70Hz(const ros::TimerEvent& event)
{
 if(odom_msg)
 {
  nav_msgs::Odometry odom70 = *odom_msg;
  pose_pub_70_Hz.publish(odom70);
 }
 return;
}


int main( int argc, char** argv)
{
  ros::init(argc, argv, "jps_server");
  JPSNodelet JPSNode;
  ROS_INFO("JPS Server Ready\n");
  ros::spin();
}
