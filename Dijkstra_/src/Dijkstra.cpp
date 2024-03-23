#include "Dijkstra.h"

using namespace std;

const int INF = numeric_limits<int>::max();

Dijkstra_cls::Dijkstra_cls(vector<vector<int>> cost_matrix_) : cost_matrix(cost_matrix_){
    node_num = cost_matrix_.size();
    distances.resize(node_num, INF);
    is_visited.resize(node_num, false);
}

Dijkstra_cls::~Dijkstra_cls()
{
}

//找到距离起始节点最近的未访问节点
int Dijkstra_cls::minDistance() {
        int MinDistance = INF, MinIndex;
        for (int i = 0; i < node_num; ++i) {
            if (!is_visited[i] && distances[i] <= MinDistance) {
                MinDistance = distances[i];
                MinIndex = i;
            }
        }
        return MinIndex;
}

//更新最短路径
vector<int> Dijkstra_cls::GetShortestPath(int start) {
    distances[start] = 0;

    for (int i = 0; i < node_num - 1; ++i) {
        int j = minDistance();
        is_visited[j] = true;

        for (int k = 0; k < node_num; ++k) {
            if (!is_visited[k] && cost_matrix[j][k] != INF && distances[j] != INF && distances[j] + cost_matrix[j][k] < distances[k]) {
                distances[k] = distances[j] + cost_matrix[j][k];
            }
        }
    }
    return distances;
}