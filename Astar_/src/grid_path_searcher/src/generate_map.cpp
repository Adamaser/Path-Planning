#include <iostream> //输入输出库
#include <Eigen/Eigen>//向量运算库文
#include <math.h>//数学运算库
#include <random>//用于生成伪随机数的类和函数

/*PCL各大头文件*/
#include <pcl/io/pcd_io.h> //用于点云数据的读取与输入，支持多种输出格式（pcb、ply、las等）
#include <pcl_conversions/pcl_conversions.h> //用于提供PCL与ROS之间点云数据类型的相互转换功能
#include <pcl/point_cloud.h>//定义点云数据结构
#include <pcl/point_types.h>//包含了一些常用的点云数据类型定义
#include <pcl/kdtree/kdtree_flann.h>//KDtree实现头文件
#include <pcl/search/kdtree.h>//PCL 自带的 KD 树实现，不依赖于 FLANN 库
#include <pcl/search/impl/kdtree.hpp>//库中 KD 树搜索的实现的内部实现头文件

/*ROS相关头文件*/
#include <ros/ros.h>//使用ROS提供的功能和API
#include <ros/console.h>//用于控制ROS节点的日志输出

/*消息类新头文件*/
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

/*命名空间*/
using namespace std;
using namespace Eigen;

/*定义发布句柄*/
ros::Publisher  _map_pub;//用于发布生成的三维点云地图

/*定义变量*/
int _obs_num, _cir_num;//定义柱体与圆的数量
double _x_size, _y_size, _z_size, _init_x, _init_y, _resolution, _sense_rate;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _w_c_l, _w_c_h;//x、y、w、h、w_c的最大最小值
bool _has_map = false;//标志地图成功生成的标志位

/*类中消息定义*/
sensor_msgs::PointCloud2 globalMap_pcd;//用于存储生成的三维点云地图的ROS消息类型。
pcl::PointCloud<pcl::PointXYZ> cloudMap;//存储生成的三维点云地图的PCL点云对象。
pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;//Kd树搜索器，用于加速后续对地图中点的搜索操作
vector<int>     pointIdxSearch;
vector<float>   pointSquaredDistance; 

void RandomMapGenerate()
{
    //创建随即设备与随机数生成引擎
    random_device rd;
    default_random_engine eng(rd());

    /*随机生成柱体的横坐标、纵坐标、底面宽度与高*/
    uniform_real_distribution<double> rand_x = uniform_real_distribution<double>(_x_l, _x_h );
    uniform_real_distribution<double> rand_y = uniform_real_distribution<double>(_y_l, _y_h );
    uniform_real_distribution<double> rand_w = uniform_real_distribution<double>(_w_l, _w_h);
    uniform_real_distribution<double> rand_h = uniform_real_distribution<double>(_h_l, _h_h);

    /*随机生成圆的横坐标、纵坐标、半径*/
    uniform_real_distribution<double> rand_x_circle = uniform_real_distribution<double>(_x_l + 1.0, _x_h - 1.0);
    uniform_real_distribution<double> rand_y_circle = uniform_real_distribution<double>(_y_l + 1.0, _y_h - 1.0);
    uniform_real_distribution<double> rand_r_circle = uniform_real_distribution<double>(_w_c_l    , _w_c_h    );

    /*随机生成旋转角度roll、pitch、yaw*/
    uniform_real_distribution<double> rand_roll      = uniform_real_distribution<double>(- M_PI,     + M_PI);//翻滚角
    uniform_real_distribution<double> rand_pitch     = uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);//俯仰角
    uniform_real_distribution<double> rand_yaw       = uniform_real_distribution<double>(+ M_PI/4.0, + M_PI/2.0);//偏航角

    /*随机生成椭圆参数*/
    uniform_real_distribution<double> rand_ellipse_c = uniform_real_distribution<double>(0.5, 2.0);
    uniform_real_distribution<double> rand_num       = uniform_real_distribution<double>(0.0, 1.0);

    /*临时变量，用于存储随机生成的点*/
    pcl::PointXYZ pt_random;

    /*生成圆形*/
    for (int i = 0; i < _cir_num; i++)
    {
        double x0, y0, z0, R;//定义临时变量存储x、y、z、半径的随机值
        std::vector<Vector3d> circle_set;//用于存储随机圆上的点

        //在定义的范围内生成随机值
        x0 = rand_x_circle(eng);
        y0 = rand_y_circle(eng);
        z0 = rand_h(eng)/2;
        R = rand_r_circle(eng);

        //判断圆心位置，如果圆心离初始点太近（距离小于2.0），则跳过本次循环，不生成障碍物
        if (sqrt(pow(x0 - _init_x, 2) + (pow(y0 - _init_y, 2))) < 2.0)
            continue;

        //随机生成作元参数a,b
        double a, b;
        a = rand_ellipse_c(eng);
        b = rand_ellipse_c(eng);

        //根据椭圆形状生成圆上的点
        double x, y, z;
        Vector3d pt3, pt3_rot;
        for (double theta = -M_PI; theta < M_PI; theta += 0.025)
        {
            x = a * cos(theta) * R;
            y = b * sin(theta) * R;
            z = 0;
            pt3 << x, y, z;
            circle_set.push_back(pt3);
        }

        //定义一个随机的3D旋转矩阵
        Matrix3d Rot;

        //定义姿态角度
        double roll, pitch, yaw;
        double alpha, beta, gama;
        roll = rand_roll(eng);   // 翻滚角
        pitch = rand_pitch(eng); // 俯仰角
        yaw = rand_yaw(eng);     // 偏航角

        alpha = roll;
        beta = pitch;
        gama = yaw;


        /*一半概率将偏航和俯仰角指定为PI/2*/
        double p = rand_num(eng);
        if (p < 0.5)
        {
            beta = M_PI / 2.0;
            gama = M_PI / 2.0;
        }

        Rot << cos(alpha) * cos(gama) - cos(beta) * sin(alpha) * sin(gama), -cos(beta) * cos(gama) * sin(alpha) - cos(alpha) * sin(gama), sin(alpha) * sin(beta),
             cos(gama) * sin(alpha) + cos(alpha) * cos(beta) * sin(gama), cos(alpha) * cos(beta) * cos(gama) - sin(alpha) * sin(gama), -cos(alpha) * sin(beta),
             sin(beta) * sin(gama), cos(gama) * sin(beta), cos(beta);
        
        /*根据旋转矩阵对圆上的点进行随机旋转，并加上圆心坐标，得到随机生成的圆形障碍物点*/
        for (auto pt : circle_set)
        {
            pt3_rot = Rot * pt;//旋转矩阵乘除向量得到旋转后的圆点云
            pt_random.x = pt3_rot(0) + x0 + 0.001;
            pt_random.y = pt3_rot(1) + y0 + 0.001;
            pt_random.z = pt3_rot(2) + z0 + 0.001;

            // 如果障碍物的高度大于等于0.0，则将障碍物点加入地图
            if (pt_random.z >= 0.0)
                cloudMap.points.push_back(pt_random);
        }

    }
    // 判断Kd树是否为空，用于后续判断是否需要进行障碍物的碰撞检测
   bool is_kdtree_empty = false;
   if (cloudMap.points.size() > 0)
      kdtreeMap.setInputCloud(cloudMap.makeShared());
   else
      is_kdtree_empty = true;

    /*放置柱形障碍物*/
    for (int i = 0; i < _obs_num; i++)
    {
        double x, y, w, h;
        x = rand_x(eng);
        y = rand_y(eng);
        w = rand_w(eng);
    

        //当柱状障碍物的位置距离障碍物太近时，结束本次循环，不生成障碍物
        if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 0.8)
            continue;
        
        //用Kd树搜索器查找当前障碍物的位置，进行碰撞检测
        pcl::PointXYZ searchPoint(x, y, (_h_l + _h_h) / 2.0);
        pointIdxSearch.clear();//清空ID与距离信息
        pointSquaredDistance.clear();

        if (is_kdtree_empty == false)
        {
        if (kdtreeMap.nearestKSearch(searchPoint, 1, pointIdxSearch, pointSquaredDistance) > 0)
            {
            // 如果当前障碍物和最近邻障碍物的距离小于1.0，则跳过本次循环，不生成障碍物
            if (sqrt(pointSquaredDistance[0]) < 1.0)
                continue;
            } 
        }
        
        // 将障碍物的坐标量化到分辨率上，并生成柱状障碍物的高度
        x = floor(x / _resolution) * _resolution + _resolution / 2.0;
        y = floor(y / _resolution) * _resolution + _resolution / 2.0;

        int widNum = ceil(w / _resolution);
        for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
        {
            for (int s = -widNum / 2.0; s < widNum / 2.0; s++)
            {
                h = rand_h(eng);
                int heiNum = 2.0 * ceil(h / _resolution);//保持精度？
                for (int t = 0; t < heiNum; t++)
                {
                    // 将柱状障碍物的各个点加入地图
                    pt_random.x = x + (r + 0.0) * _resolution + 0.001;
                    pt_random.y = y + (s + 0.0) * _resolution + 0.001;
                    pt_random.z = (t + 0.0) * _resolution * 0.5 + 0.001;
                    cloudMap.points.push_back(pt_random);
                }
            }
        }
    }
    // 设置点云地图的属性
   cloudMap.width = cloudMap.points.size();
   cloudMap.height = 1;
   cloudMap.is_dense = true;

   // 标记已经生成了地图
   _has_map = true;

   // 将pcl::PointCloud转换为sensor_msgs::PointCloud2消息，并设置header的frame_id为"world"
   pcl::toROSMsg(cloudMap, globalMap_pcd);
   pcl::io::savePCDFileASCII("/home/chen/UAV_class_ws/Astar/src/grid_path_searcher/pcd/map.pcd", cloudMap);
   globalMap_pcd.header.frame_id = "world";
}

/*将地图信息通过话题发送出去*/
void pubSensedPoints()
{
    if( !_has_map) return;
    _map_pub.publish(globalMap_pcd);
}

/*主函数，用以ROS初始化、参数赋值、节点定义等*/
int main (int argc, char** argv)
{
    ros::init (argc, argv, "random_complex_scene");
    ros::NodeHandle n( "~" );//创建节点的私有命名空间
    
    _map_pub  = n.advertise<sensor_msgs::PointCloud2>("global_map", 1);

   n.param("init_state_x", _init_x,       0.0);
   n.param("init_state_y", _init_y,       0.0);

   n.param("map/x_size",  _x_size, 50.0);
   n.param("map/y_size",  _y_size, 50.0);
   n.param("map/z_size",  _z_size, 5.0 );

   n.param("map/obs_num",    _obs_num,  30);
   n.param("map/circle_num", _cir_num,  30);
   n.param("map/resolution", _resolution, 0.2);

   n.param("ObstacleShape/lower_rad", _w_l,   0.3);
   n.param("ObstacleShape/upper_rad", _w_h,   0.8);
   n.param("ObstacleShape/lower_hei", _h_l,   3.0);
   n.param("ObstacleShape/upper_hei", _h_h,   7.0);

   n.param("CircleShape/lower_circle_rad", _w_c_l, 0.3);
   n.param("CircleShape/upper_circle_rad", _w_c_h, 0.8);

   n.param("sensing/rate", _sense_rate, 1.0);

   _x_l = - _x_size / 2.0;
   _x_h = + _x_size / 2.0;

   _y_l = - _y_size / 2.0;
   _y_h = + _y_size / 2.0;

   RandomMapGenerate();

   ros::Rate loop_rate(_sense_rate);//控制运行频率，每秒_sense_rate次
   while (ros::ok())//只要不打断一直运行
   {
      pubSensedPoints();
      ros::spinOnce();//及时处理受到的信息
      loop_rate.sleep();//控制主频率，确保数据按照指定的频率发送，并且不会过于频繁地处理消息队列
   }
}