#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "Astar.h"
#include "backward.hpp"
#include <vector>
#include "planner.h"

using namespace std;
using namespace Eigen;

namespace backward {
backward::SignalHandling sh;
}


nav_msgs::Path path_desire;
//先实例化一个geometry_msgs::PoseStamped类型的对象，并对其赋值，最后将其发布出去
geometry_msgs::PoseStamped pose;
//定义三维向量分别记录位置、速度、加速度
Eigen::Vector3d pos_desire;


//分辨率、分辨率倒数、未知？？？
double _resolution, _inv_resolution, _cloud_margin;

//世界坐标系(单位m)下，整个点云地图的长、宽、高
double _x_size, _y_size, _z_size;    

//栅格坐标系(与世界坐标系相差分辨率)下，整个点云地图的长、宽、高
int _max_x_id, _max_y_id, _max_z_id;

//起点坐标 世界坐标系(单位m)
Vector3d _start_pt;

//地图三轴最小和最大尺寸 世界坐标系(单位m)
Vector3d _map_lower, _map_upper;

std::string _distance;
double _weight_a,_weight_b;

//将障碍物点云转换格式
std::vector<Vector3d> obs_grid_point;


ros::Subscriber _map_sub;               //点云地图的接收者
ros::Subscriber _pts_sub;               //终点坐标的接收者
ros::Publisher  _grid_path_vis_pub;     //发布Astar找到的路径
ros::Publisher  _grid_obs_vis_pub;     //发布障碍物珊格
ros::Publisher  _visited_nodes_vis_pub; //发布OpenList/CloseList的方格
ros::Publisher  _grid_map_vis_pub;      //发布点云地图
ros::Publisher  _path_A_star_pub;       //发布初始A*算法路径

ros::Publisher _path_minimum_pub;//发布minimum-snap轨迹

//Astar算法的对象指针
AstarPathFinder * _astar_path_finder     = NULL;

//标志位，确保先有地图再有终点坐标
bool _has_map   = false;

void pathFinding(const Vector3d start_pt, const Vector3d target_pt);
void visGridObs(vector<Vector3d> nodes);
// void get_information_way(vector<Vector3d> nodes);
// void generate_opt_path();
// void show_opt_path();


//终点坐标的回调函数
void rcvWaypointsCallback(const nav_msgs::Path & wp)
{    
    //安全性检查 
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
        return;
    //目标点的三维值  世界坐标系
    Vector3d target_pt;
    target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;

    ROS_INFO("[node] receive the planning target");
    pathFinding(_start_pt, target_pt); 
    _start_pt = target_pt;
}

//点云地图的回调函数
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    // std::cout<<"进入回调函数"<<endl;
    //地图生成标志位
    if(_has_map) return;

    //PCL点云格式
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;

    //ROS点云格式
    sensor_msgs::PointCloud2 map_vis;

    //将ROS点云格式转换为PCL格式
    pcl::fromROSMsg(pointcloud_map, cloud);

    //当点云数据为空的时候直接返回
    if ((int)cloud.size() == 0) return;

    pcl::PointXYZ pt;//世界坐标的三维点

     //将含有点云数据的地方记录为障碍物
    for (int idx = 0; idx < (int)cloud.points.size(); idx++){
        pt = cloud.points[idx];

        //点云坐标位置设置为障碍物
        _astar_path_finder -> setObs(pt.x, pt.y, pt.z);

        //坐标归一化后的点,用于显示出来
        Vector3d cor_round = _astar_path_finder->coordRounding(Vector3d(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }
    

    // int count=0;
    // int i=0 ;
    for (const auto& point : cloud_vis.points){
        // std::cout << point << endl;
        // std::cout << count << endl;
        // count++;
        obs_grid_point.push_back(Vector3d(point.x, point.y, point.z));
        // std::cout << obs_grid_point[i++]<<endl;
    }

    cloud_vis.width = cloud_vis.points.size();//点云数
    cloud_vis.height = 1;//点云高度
    cloud_vis.is_dense = true;//所有点都为有效点
    
    //计算完成PCL格式转ROS格式，将cloud_is用于点云显示
    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "/world";//确定坐标系，此处采用世界坐标
    
    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
}

//可视化路径函数
void visGridPath(vector<Vector3d> nodes){
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();

    node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}

//可视化障碍物珊格
void visGridObs(vector<Vector3d> nodes){
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();

    node_vis.ns = "demo_node/obs_grid";

    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 1.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _grid_obs_vis_pub.publish(node_vis);
}

//可视化初始astar路径（未优化路径）
void visAstarpath(vector<Vector3d> nodes){
    // Create a Marker message
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world"; // Set the frame ID to match your RViz frame
    line_strip.header.stamp = ros::Time::now();
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.id = 0;
    line_strip.scale.x = 0.05; // Line width

    // Set the color of the line (green in this example)
    line_strip.color.r = 0.0;
    line_strip.color.g = 1.0;
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0; // Fully opaque

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        line_strip.points.push_back(pt);
    }
    _path_A_star_pub.publish(line_strip);
}

// 发布OpenList/CloseList的方格
void visVisitedNode(vector<Vector3d> nodes)
{
    visualization_msgs::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/expanded_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;
    cout<<nodes.size();

    geometry_msgs::Point pt;
    for (int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}

//进行minimum-snap优化并且可视化路径
void CalShow_minimum_tra(vector<Vector3d> nodes){
    nav_msgs::Path tra_new;
    tra_new.header.frame_id = "world";
    tra_new.header.stamp = ros::Time::now();
    planner planner(nodes);
    tra_new = planner.tra;
    _path_minimum_pub.publish(tra_new);

}

//路径查找，主要调用Astar的部分接口函数
void pathFinding(const Vector3d start_pt, const Vector3d target_pt){
    cout<<"当前在pathFinding"<< endl;
    //构建A*节点并进行路径搜索
    _astar_path_finder -> AstarGraphSearch(start_pt, target_pt);


    //通过构建好的A*节点与结果得到路径与closeList点
    auto grid_path        = _astar_path_finder->getPath();
    auto visited_nodes  = _astar_path_finder->getVisitedNodes();

    //可视化路径与closelist
    visGridPath (grid_path);
    //发布由A*算法生成的路径
    visAstarpath (grid_path);
    //发布障碍物珊格数据
    visGridObs(obs_grid_point);
    //轨迹minimum-snap算法
    CalShow_minimum_tra(grid_path);
    //重置Astar算法，用于再次调用
    _astar_path_finder -> resetUsedGrids();
}

// void path_vector_Finding(const vector<Vector3d> view_point){
//     count<<"已经传入路径点，现在对路径点进行规划"
//     待补充,读取当前位置记为start_pt
//     pathFinding(start_pt, first_target);
//     int num_viewpoint = view_point.size();
//     Vector3i temp_id = [0,0,0];
//     for(int i = 1; i < num_viewpoint; i++){
//         temp_id = _astar_path_finder -> coord2gridIndex(view_point[i]);
//         if (_astar_path_finder->isOccupied(temp_id[0],temp_id[1],temp_id[2])){
//             continue;
//         }
//         while (!arrive()) 补充到达判定函数
//          {
//            pathFinding(view_point[i-1],view_point[i])
//          }  
//     }
// }

//主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar_demo");
    ros::NodeHandle nh("~");

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );
    _grid_map_vis_pub         = nh.advertise<sensor_msgs::PointCloud2>("point_map", 1);
    _grid_path_vis_pub        = nh.advertise<visualization_msgs::Marker>("astar_path", 1);
    _grid_obs_vis_pub         = nh.advertise<visualization_msgs::Marker>("obs_grid", 1);
    _visited_nodes_vis_pub        = nh.advertise<sensor_msgs::PointCloud2>("visited_nodes", 1);
    _path_A_star_pub          = nh.advertise<visualization_msgs::Marker>("vis_a_star", 1);
    _path_minimum_pub         = nh.advertise<nav_msgs::Path>("/tra_generation", 10); 



    //加载配置参数
    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map_resolution", _resolution, 0.2);
    nh.param("map/x_size",        _x_size, 10.0);
    nh.param("map/y_size",        _y_size, 10.0);
    nh.param("map/z_size",        _z_size, 4.0 );
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  1.0);

    nh.param("heuristic/distance", _distance, string("euclidean"));
    nh.param("weight/a", _weight_a, 1.0);
    nh.param("weight/b", _weight_b, 1.0);

    //分辨率倒数
    _inv_resolution = 1.0 / _resolution;

    //通过加载的数据得到_map_lower、_map_upper
    _map_lower << - _x_size/2.0, - _y_size/2.0,    0.0 ;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;

    //通过加载的数据得到_max_x_id、_max_y_id、_max_z_id
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    //构建astar指针对象并初始化
    _astar_path_finder = new AstarPathFinder(_distance, _weight_a, _weight_b);

    //初始化指针地图为珊格地图
    _astar_path_finder -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);

    //ROS循环主函数
    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }

    //释放new分配的空间
    delete _astar_path_finder;
    return 0;

}
