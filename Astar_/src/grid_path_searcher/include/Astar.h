#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <vector>
#include "backward.hpp"
#include "node.h"

class AstarPathFinder
{	
	private:

	protected:
	    std::vector<int>data;
		std::vector<std::vector<std::vector<GridNodePtr> > >GridNodeMap;
		//uint8_t * data;   //按照Z->Y->X顺序，构建一维点云栅格数据存储格式
		//GridNodePtr *** GridNodeMap;  //构建3维数组，用于判断存储Astar节点
		Eigen::Vector3i goalIdx;      //目标点

		//栅格坐标系(与世界坐标系相差分辨率)下，整个点云地图的宽、长、高
		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;  
		int GLXYZ_SIZE, GLYZ_SIZE;

        //分辨率、分辨率倒数
		double resolution, inv_resolution;

		//世界坐标系下点云地图x、y、z轴最小和最大尺寸
		double gl_xl, gl_yl, gl_zl;
		double gl_xu, gl_yu, gl_zu;
         
        std::string distance;
		double weight_a,weight_b;

		//终点节点，用于路径的反向查找
		GridNodePtr terminatePtr;

		//openList列表
		std::multimap<double, GridNodePtr> openSet;
        
		//计算Astar的启发式H值
		double calHeu(GridNodePtr node1, GridNodePtr node2);

		//寻找Astar中某点的邻居点集
		void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);		

        //障碍物判断
    	bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
		
		//栅格转世界
		Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);

		//世界转栅格
		Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);

	public:
		AstarPathFinder(std::string _distance,double _weight_a,double _weight_b);
		~AstarPathFinder(){};
		//初始化点云地图
		void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
		
		//Astar的核心函数
		void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
        
		//设置一维点云数据格式中的障碍物的位置
		void setObs(const double coord_x, const double coord_y, const double coord_z);
     
	    //获取Astar的路径
		std::vector<Eigen::Vector3d> getPath();

		//获取closelist中点的世界坐标下的索引
		std::vector<Eigen::Vector3d> getVisitedNodes();
        
		//Astar节点重置
		void resetGrid(GridNodePtr ptr);
		void resetUsedGrids();

		Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);


};

#endif
