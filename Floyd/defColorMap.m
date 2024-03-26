%创建一个函数定义颜色图，并初始化障碍物
function [field,grid_color] = defColorMap(rows, cols)
grid_color = [1 1 1;%白色——自由栅格
    0 0 0;%黑色——障碍物
    1 0 0;%红色——移动障碍物
    1 1 0;%黄色——起点
    1 0 1;%紫色——目标点
    0 1 0;%绿色——路径搜索结果
    0 1 1];%蓝色——动态规划路径

% 构建颜色图
colormap(grid_color);

%定义栅格地图全域，初始化空白区域
field = ones(rows, cols);

% 障碍物区域初始化
obsRate = 0.3;
obsNum = floor(rows*cols*obsRate);
obsIndex = randi([1,rows*cols],obsNum,1);
field(obsIndex) = 2;

