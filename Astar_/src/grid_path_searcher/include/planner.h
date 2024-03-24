// #ifndef ROS_NODE.H
// #define ROS_NODE.H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
 #include "trajectory_generator.h"
using namespace std;
using namespace Eigen;
#define snap 4;
#define jerk 3;


class  TrajectoryGeneratorTool;
class planner
{
    private:
        ros::NodeHandle n ;
        int  dot_num ;      //保存约束点个数       
        Eigen::MatrixXd route;          //矩阵保存约束点
        Eigen::VectorXd time ;               //向量保存轨迹时间
        Eigen::MatrixXd poly_coeff;     //   系数矩阵
        int poly_coeff_num;
        int mode = snap;
        
    public:
        nav_msgs::Path tra;
        planner()                               //构造函数；
        {
            poly_coeff = planner::getcoeff();   
            // planner::getPosPoly(poly_coeff , 2 , 0.05);
            planner::trajectory_path();
            planner::tra_publish();
        }
        //为A*算法定制的构造函数
        planner(vector<Vector3d> nodes){
            poly_coeff = planner::getcoeff(nodes);   
            planner::trajectory_path();
            tra = planner::tra_get_info();
        }

        
        // friend class TrajectoryGeneratorTool;   //  友元类
        void getparam(void);    //获取参数
        void getpara_node(vector<Vector3d> nodes);//从路径点中获取参数
        Eigen::MatrixXd getcoeff(void);
        Eigen::MatrixXd getcoeff(vector<Vector3d> nodes);
        Eigen::Vector3d getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t) ;
        nav_msgs::Path trajectory_path(void);
        nav_msgs::Path tra_get_info(void);
        void tra_publish(void);
};

// #endif