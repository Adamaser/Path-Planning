#ifndef _NODE_H_
#define _NODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"

#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{     
    int id;        // 1表示openlist, -1表示closelist  0表示即不在open 也不在close中
    Eigen::Vector3d coord; 
    Eigen::Vector3i dir;   // direction of expanding
    Eigen::Vector3i index;
	
    double gScore, fScore;
    GridNodePtr cameFrom;
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
		id = 0;
		index = _index;     //栅格索引
		coord = _coord;     //栅格索引所对应的点云位置
		dir   = Eigen::Vector3i::Zero();

		gScore = inf;          //astar g值
		fScore = inf;          //astar f值
		cameFrom = NULL;       //父节点
    }

    GridNode(){};
    ~GridNode(){};
};


#endif
