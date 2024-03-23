#include <iostream>
#include <Dijkstra.h>


int main() {
    const int INF = numeric_limits<int>::max();
    vector<vector<int>> cost_matrix = {
        {0, 1, 4, INF, INF},
        {1, 0, 4, 2, 7},
        {4, 4, 0, 3, INF},
        {INF, 2, 3, 0, 4},
        {INF, 7, INF, 4, 0}
    };

    Dijkstra_cls dijkstra(cost_matrix);
    vector<int> shortestDistances = dijkstra.GetShortestPath(0);

    cout << "Shortest distances from node 0:" << endl;
    for (int i = 0; i < shortestDistances.size(); ++i) {
        if (shortestDistances[i] == INF) {
            cout << "Node " << i << ": " << "Not reachable" << endl;
        } else {
            cout << "Node " << i << ": " << shortestDistances[i] << endl;
        }
    }
    return 0;
}