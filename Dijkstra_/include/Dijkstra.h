#ifndef _DIJKSTRA_H_
#define _DIJKSTRA_H_

#include <iostream>
#include <vector>
#include <limits>

using namespace std;

class Dijkstra_cls
{
private:
    //定义一个存储双向代价图的矩阵
    vector<vector<int>> cost_matrix;
    //定义存储每个节点距离起点最小距离的容器
    vector<int> distances;
    //定义存储当前节点探索状态标志位容器
    vector<bool> is_visited;
    //定义存储节点个数的变量
    int node_num;

    int minDistance();

public:
    Dijkstra_cls(vector<vector<int>> cost_matrix_);
    ~Dijkstra_cls();
    vector<int> GetShortestPath(int start);
};


#endif