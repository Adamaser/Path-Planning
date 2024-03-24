# grid_path_searcher
1. pcd文件修改：

    点云信息通过pcd文件读取，地图发布节点文件位于grid_path_searcher/src/complex_map.cpp，修改main中的pcd文件路径即可

2. A*算法调用：

    （1）相关文件：
    包含头文件：
    grid_path_searcher/include/Astar_searcher.h
    grid_path_searcher/include/Astar.h
    grid_path_searcher/include/backward.hpp
    grid_path_searcher/include/node.h
    添加cpp文件：
    grid_path_searcher/src/Astar.cpp
    grid_path_searcher/src/complex_map.cpp

    （2）调用方法：
    如grid_path_searcher/src/astar_demo.cpp文件中的main所示,直接复制过去就好，初始化一些参数，可以直接定义或者读取配置文件
    //实例化，并参数构造AstarPathFinder类
    AstarPathFinder * _astar_path_finder 
    //构建astar指针对象并初始化
    _astar_path_finder = new AstarPathFinder(_distance, _weight_a, _weight_b);
    //初始化指针地图为珊格地图
    _astar_path_finder -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    ！！！用一次记得delete释放内存
    //调用函数
    //构建A*节点并进行路径搜索
    _astar_path_finder -> AstarGraphSearch(start_pt, target_pt);
    //通过构建好的A*节点与结果得到路径
    auto grid_path        = _astar_path_finder->getPath();
3. minimum-snap算法优化
    （1）相关文件
    包含头文件：grid_path_searcher/include/planner.h、grid_path_searcher/include/trajectory_generator.h
    添加cpp文件：grid_path_searcher/src/trajectory_generator.cpp、grid_path_searcher/src/planner.cpp
    （2）调用方法（参考grid_path_searcher/src/astar_demo.cpp中的CalShow_minimum_tra函数）：
    实例化类 planner planner(nodes);//node为A*中找到的路径点grid_path
    planner.tra中存储的即为优化后的曲线




