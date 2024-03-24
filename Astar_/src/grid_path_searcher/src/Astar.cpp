#include "Astar.h"
#include "functional"

using namespace std;
using namespace Eigen;


/**************************************TODO1 A*算法相关函数****************************************/

//定义一个std::function类名为Distance的对象，输入为Eigen的三维向量，返回值类型为double
std::function<double(Eigen::Vector3d, Eigen::Vector3d)>Distance;

/***
*@函数功能  计算两点之间的欧式距离
----------------------------------------------
*@参数     node1_coord     节点1点云位置
*@参数     node2_coord     节点2点云位置
----------------------------------------------
*@返回值   欧式距离：根号√((x2-x1)^2+(y2-y1)^2)
***/
double EuclideanDistance(Eigen::Vector3d node1_coord,Eigen::Vector3d node2_coord){
    double h = std::sqrt(std::pow(node1_coord(0) - node2_coord(0), 2 ) +
               std::pow(node1_coord(1) - node2_coord(1), 2 ) +
               std::pow(node1_coord(2) - node2_coord(2), 2 ));
    return h;
}

/***
*@函数功能  计算两点之间的曼哈顿距离
----------------------------------------------
*@参数     node1_coord     节点1点云位置
*@参数     node2_coord     节点2点云位置
----------------------------------------------
*@返回值   曼哈顿距离值：|(x2-x1)+(y2-y1)|
***/
double ManhattanDistance(Eigen::Vector3d node1_coord,Eigen::Vector3d node2_coord){
    double h = std::abs(node1_coord(0) - node2_coord(0)) + 
               std::abs(node1_coord(1) - node2_coord(1)) + 
               std::abs(node1_coord(2) - node2_coord(2));
    return h;
}

/***
*@函数功能  构造函数
----------------------------------------------
*@参数     _distance     Astar算法距离启发函数
*@参数     _weight_a     Astar算法的权重a
*@参数     _weight_b     Astar算法的权重b
***/
AstarPathFinder::AstarPathFinder(std::string _distance, double _weight_a, double _weight_b) : distance(_distance),weight_a(_weight_a),weight_b(_weight_b){
    Distance = EuclideanDistance;
    if (distance == "euclidean"){
        std::cout << "use euclidean distance" << std::endl;
        Distance = EuclideanDistance;
    }
    else if (distance == "manhattan"){
        std::cout << "use manhattan distance" << std::endl;
        Distance = ManhattanDistance;
    }
    std::cout<<"weight a:"<<weight_a<<" weight b:"<<weight_b<<std::endl;
}

/**************************************TODO2 地图处理相关函数****************************************/

/***
 *@函数功能 初始化点云地图,建立珊格地图
 --------------------------------------------
 参数：
 _resolution    分辨率
 global_xyz_l   世界坐标系下点云地图三轴最小尺寸
 global_xyz_u   世界坐标系下点云地图三轴最小尺寸
 max_x_id       栅格坐标系(与世界坐标系相差分辨率)下，整个点云地图的宽
 max_y_id       栅格坐标系(与世界坐标系相差分辨率)下，整个点云地图的长
 max_z_id       栅格坐标系(与世界坐标系相差分辨率)下，整个点云地图的高
 ***/
void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id){
    
    gl_xl = global_xyz_l(0);//世界坐标系下点云地图x轴的最小值
    gl_yl = global_xyz_l(1);//世界坐标系下点云地图y轴的最小值
    gl_zl = global_xyz_l(2);//世界坐标系下点云地图z轴的最小值

    gl_xu = global_xyz_u(0);//世界坐标系下点云地图x轴的最大值
    gl_yu = global_xyz_u(1);//世界坐标系下点云地图y轴的最大值
    gl_zu = global_xyz_u(2);//世界坐标系下点云地图z轴的最大值

    GLX_SIZE = max_x_id;//珊格坐标系下的x轴坐标的最大值
    GLY_SIZE = max_y_id;//珊格坐标系下的x轴坐标的最大值
    GLZ_SIZE = max_z_id;//珊格坐标系下的x轴坐标的最大值

    GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;//地图的分辨率
    inv_resolution = 1.0 / _resolution;//地图分辨率的倒数

    data.resize(GLXYZ_SIZE * sizeof(uint8_t),0);//构建数组存储点云珊格数据
    GridNodeMap.resize(GLX_SIZE);
    for (int i=0; i<GLX_SIZE; i++){
        GridNodeMap[i].resize(GLY_SIZE);
        for (int j=0; j<GLY_SIZE; j++){
            GridNodeMap[i][j].resize(GLZ_SIZE);
            for (int k=0; k<GLZ_SIZE; k++){
                Vector3i tmpIdx(i,j,k); //定义珊格索引
                Vector3d pos = gridIndex2coord(tmpIdx); //珊格对应的世界坐标系
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }

    }
}

/***
 *@函数功能 标记障碍物
 --------------------------------------------
 参数：
 coord_x       世界坐标系下，某个障碍物点云的x值
 coord_y       世界坐标系下，某个障碍物点云的y值
 coord_z       世界坐标系下，某个障碍物点云的z值
 ***/
void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z){
    //边界检查
    if( coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl ||
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
       return;
    //世界坐标系转珊格坐标系
    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);
    //标记为障碍物，数据按照Z>Y>X的顺序排序，1表示该珊格处为障碍物
     data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

/***
 *@函数功能 珊格坐标系转世界坐标系
 --------------------------------------------
 参数：
 index    栅格索引
 返回值：  该栅格所定应的世界坐标系下点云位置坐标
 ***/
Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index){
    Vector3d pt;//用以暂时存储世界坐标下坐标

    pt(0) = ((double)index(0) + 0.5 ) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5 ) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5 ) * resolution + gl_zl;

    return pt;
}

/***
 *@函数功能 珊格坐标系转世界坐标系
 --------------------------------------------
 参数：
 pt       世界坐标系下点云索引
 返回值：  对应的栅格坐标系的栅格索引
 ***/
Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt){
    Vector3i idx;//暂时存储珊格坐标系下的坐标值

    idx << min(max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1 ),
           min(max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1 ),
           min(max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1 );

    return idx;
}

/***
 *@函数功能 将世界坐标系归一化到珊格中间
 --------------------------------------------
 参数：
 coord       世界坐标系下点云索引
 返回值       世界坐标系下的坐标
 ***/
Vector3d AstarPathFinder::coordRounding(const Vector3d & coord){
    return gridIndex2coord(coord2gridIndex(coord));
}

/**************************************TODO3 A*本体函数**************************************/

/***
 *@函数功能 Astar核心函数
 --------------------------------------------
 参数：
 start_pt   世界坐标系下点云中起点位置
 end_pt     世界坐标系下点云中终点位置
***/
void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt){
    //记录开始时间
    ros::Time time_1 = ros::Time::now();

    //起点与目标点的珊格坐标系
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //珊格转世界坐标系，规范化后的世界坐标系坐标
    start_pt = gridIndex2coord(start_idx);
    end_pt = gridIndex2coord(end_idx);

    //初始化起点与终点珊格
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr = new GridNode(end_idx, end_pt);

    //算法开始时清空openset，类型为std::multimap<double, GridNodePtr>，拥有默认排序功能
    openSet.clear();

    //清空当前节点与邻居节点对象
    GridNodePtr currentPtr = NULL;
    GridNodePtr neighborPtr = NULL;

    //将起点加入到openlist中
    startPtr -> gScore = 0;
    startPtr -> fScore = weight_b * calHeu(startPtr,endPtr);
    startPtr -> id = 1;
    startPtr -> coord = start_pt;

    //键为f值，值为该节点，multimap可以按照键值从小到大自动排序
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );

    // 将起点的栅格索引标记为openlist
    GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]] -> id = 1;

    Vector3i current_idx;
    vector<GridNodePtr> neighborPtrSets; //定义存储邻居节点的容器
    vector<double> edgeCostSets;//定义存储代价的容器

    //Astar循环搜索过程
    while (!openSet.empty()){
        //通过取multimap刚开始键所定义值，就表示f值最小的节点，当然刚开始currentPtr就是起点
        currentPtr = openSet.begin() -> second;
        //从Openset中移除该点
        openSet.erase(openSet.begin());
        //记录当前节点的珊格索引
        current_idx = currentPtr->index;
        //将当前点的栅格索引标记为closelist  表示该点已经访问过
        GridNodeMap[current_idx[0]][current_idx[1]][current_idx[2]] -> id = -1;

        //判断到达终点时，打印算法所需时间
        if(currentPtr->index == goalIdx){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }

        //查询当前节点的所有邻居节点
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        //遍历所有的邻居节点
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            neighborPtr = neighborPtrSets[i];
            //0表示该点不再openlist中，需要将点加入到openlist中
            if(neighborPtr -> id == 0 ){
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = weight_a * neighborPtr->gScore + weight_b * calHeu(neighborPtr,endPtr);
                neighborPtr->cameFrom = currentPtr;
                openSet.insert(make_pair(neighborPtr -> fScore,neighborPtr));
                neighborPtr -> id = 1;
            }
            else if(neighborPtr -> id == 1){
                //对当前代价进行更新
                if(neighborPtr -> gScore > (currentPtr->gScore + edgeCostSets[i])){
                    neighborPtr -> gScore = currentPtr -> gScore + edgeCostSets[i];
                    neighborPtr -> fScore = weight_a*neighborPtr -> gScore +  weight_b*calHeu(neighborPtr,endPtr);
                    neighborPtr -> cameFrom = currentPtr;
                }
                continue;
            }
            //已经访问过，直接跳过
            else continue;
        }
    }
    ros::Time time_2 = ros::Time::now();
    if((time_2-time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}

/***
 *@函数功能 计算Astar的启发式H值
 --------------------------------------------
 参数：
 node1     世界坐标系下点云中点1的指针对象
 node2     世界坐标系下点云中点2的指针对象 
 返回值：   世界坐标系下两点之间的H值
 ***/
double AstarPathFinder::calHeu(GridNodePtr node1, GridNodePtr node2){
    double h;
    Vector3d node1_coord = node1->coord;
    Vector3d node2_coord = node2->coord;
    h = Distance(node1_coord, node2_coord);
    return h;
}

/***
 *@函数功能 查看当前节点的邻居节点点集
 --------------------------------------------
 参数：
 currentPtr       当前节点
 neighborPtrSets  合适邻居节点
 edgeCostSets     该邻居点与当前点之间的代价值（欧式距离）
 ***/
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets){
    neighborPtrSets.clear();
    edgeCostSets.clear();

    if(currentPtr == nullptr)
        std::cout << "Error: Current pointer is null " << endl;
    
    Vector3i thisNode = currentPtr -> index;
    int this_x = thisNode[0];
    int this_y = thisNode[1];
    int this_z = thisNode[2];

    auto this_coord = currentPtr -> coord;
    int n_x, n_y, n_z;//邻居节点珊格坐标系下的索引
    double dist;
    GridNodePtr temp_ptr = nullptr;
    Vector3d n_coord;

    for(int i = -1; i <= 1; ++i){
        for(int j = -1; j <= 1; ++j){
            for(int k = -1; k <= 1; ++k){

                //排除当前点
                if (i == 0 && j == 0 && k == 0)
                    continue;
                //记录当前索引    
                n_x = this_x + i;
                n_y = this_y + j;
                n_z = this_z + k;
                //排除边界点
                if( (n_x < 0) || (n_y < 0) || (n_z < 0) || (n_x > (GLX_SIZE - 1)) || (n_y > (GLY_SIZE - 1)) || (n_z > (GLZ_SIZE - 1)))
                    continue;
                //排除障碍点
                if (isOccupied(n_x, n_y, n_z))
                    continue;
                
                temp_ptr = GridNodeMap[n_x][n_y][n_z];

                //排除在closelist里面的点
                if(temp_ptr->id == -1)
                    continue;
                
                n_coord = temp_ptr->coord;
                
                
                if(temp_ptr == currentPtr)
                {
                    std::cout << "Error: temp_ptr == currentPtr)" << std::endl;
                }
                
                //忽略坐标相近的点在不同珊格中的近邻点
                if( (std::abs(n_coord[0] - this_coord[0]) < 1e-6) and (std::abs(n_coord[1] - this_coord[1]) < 1e-6) and (std::abs(n_coord[2] - this_coord[2]) < 1e-6 ))
                {
                    std::cout << "Error: Not expanding correctly!" << std::endl;
                    std::cout << "n_coord:" << n_coord[0] << " "<<n_coord[1]<<" "<<n_coord[2] << std::endl;
                    std::cout << "this_coord:" << this_coord[0] << " "<<this_coord[1]<<" "<<this_coord[2] << std::endl;

                    std::cout << "current node index:" << this_x << " "<< this_y<<" "<< this_z << std::endl;
                    std::cout << "neighbor node index:" << n_x << " "<< n_y<<" "<< n_z << std::endl;
                }
                

                //计算当前点与邻居点之间的代价值
                dist = std::sqrt( (n_coord[0] - this_coord[0]) * (n_coord[0] - this_coord[0])+
                        (n_coord[1] - this_coord[1]) * (n_coord[1] - this_coord[1])+
                        (n_coord[2] - this_coord[2]) * (n_coord[2] - this_coord[2]));
                neighborPtrSets.push_back(temp_ptr); 
                edgeCostSets.push_back(dist); 

            }
        }
    }
}

/***
 *@函数功能 障碍物判断
 --------------------------------------------
 参数：
 idx_x    栅格坐标系下某点的x值
 idx_y    栅格坐标系下某点的y值
 idx_z    栅格坐标系下某点的z值
 返回值：  true是障碍物  false不是障碍物
 ***/
inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z)const{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}


/***
 *@函数功能 获取A*路径
 --------------------------------------------
 返回值     path为全局坐标系下的路径
 ***/
vector<Vector3d> AstarPathFinder::getPath(){
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    auto ptr = terminatePtr;
    while (ptr -> cameFrom != NULL){
        gridPath.push_back(ptr);
        ptr = ptr -> cameFrom;
    }

    for (auto ptr : gridPath){
        path.push_back(ptr->coord);
    }

    reverse(path.begin(), path.end());
    return path;
}

/***
 *@函数功能 获取closelist中点的世界坐标下的索引
 --------------------------------------------
 返回值     visited_nodes为closeList中点的全局坐标系
 ***/
vector<Vector3d> AstarPathFinder::getVisitedNodes()
{
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){
                if (GridNodeMap[i][j][k]-> id ==-1)
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }   
            
    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

/***
 *@函数功能 珊格属性重置
 --------------------------------------------
 参数：
 ptr       指向GridNode的指针
 ***/
void AstarPathFinder::resetGrid(GridNodePtr ptr){
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}
void AstarPathFinder::resetUsedGrids(){
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}