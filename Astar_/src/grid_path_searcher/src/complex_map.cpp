#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <random>

using namespace std;

ros::Publisher _all_map_pub;

bool _has_map  = false;
double _sense_rate;
sensor_msgs::PointCloud2 globalMap_pcd;   
std::string pcdPath;

void RandomMapGenerate()
{  
   pcl::PointCloud<pcl::PointXYZ> cloudMap;   
   pcl::io::loadPCDFile(pcdPath,cloudMap);
   _has_map = true;
   pcl::toROSMsg(cloudMap, globalMap_pcd);
   globalMap_pcd.header.frame_id = "world";
}

void pubSensedPoints()
{     
   if( !_has_map ) return;

   _all_map_pub.publish(globalMap_pcd);
}

int main (int argc, char** argv) 
{        
   ros::init (argc, argv, "complex_map");
   ros::NodeHandle n( "~" );

   _all_map_pub   = n.advertise<sensor_msgs::PointCloud2>("global_map", 1);                      

   n.param("pcd_path", pcdPath,string("/home/chen/UAV_class_ws/Astar/src/grid_path_searcher/pcd/real_pcd.pcd"));
   
   n.param("sensing/rate", _sense_rate, 1.0);

   RandomMapGenerate();
   ros::Rate loop_rate(_sense_rate);
   while (ros::ok())
   {
      pubSensedPoints();
      ros::spinOnce();
      loop_rate.sleep();
   }
}
