% ����դ���ͼ�Ļ�����·���滮�㷨
% ��4�ڣ�RRT�㷨
clc
clear
close all

%% �ϰ���հ�������ʼ�㡢Ŀ��㶨��

% ����������
rows = 30;
cols = 50;
[field,cmap] = defColorMap(rows, cols);

% ��㡢�յ㡢�ϰ�������
startPos = 2;
goalPos = rows*cols-2;
field(startPos) = 4;
field(goalPos) = 5;


%% �㷨

% �������ڵ㣬��һ�зŽڵ��ţ��ڶ��зŸýڵ�ĸ��ڵ�
treeNodes = [startPos, 0];

while true
    % ��ʼ��parentNode��childNode
    parentNode = [];
    childNode = [];
    
    % �ڵ�ͼ�ռ������������
    samplePoint = getSamplePoint(field, treeNodes);
    
    % ���α���ÿһ�����ڵ㵽������ľ��룬ȡ��Сֵ��Ӧ�����ڵ�
    for i = 1:size(treeNodes,1)
        [row_treeNode, col_treeNode] = ind2sub([rows, cols], treeNodes(i,1));
        [row_samplePoint, col_samplePoint] = ind2sub([rows, cols], samplePoint);
        dist(i) = norm([row_treeNode, col_treeNode] - [row_samplePoint, col_samplePoint]);
    end
    [~,idx] = min(dist);
    parentNode = treeNodes(idx,1);
    
    % �����µ��ӽڵ�,��������
    childNode = getChildNode(field, parentNode, samplePoint);
    
    % �жϸ��ӽڵ��Ƿ񳬹���ͼ����
    if childNode(1) < 1 || childNode(1) > rows ||...
            childNode(2) < 1 || childNode(2) > cols
        continue
    else
        % תΪ��������
        childNode = sub2ind([rows, cols], childNode(1), childNode(2));
    end

    
    % �жϸ��ڵ����ӽڵ�������Ƿ����ϰ���
    flag = judgeObs(field, parentNode, childNode);
    if flag
        continue
    end
    
    % �жϸ��ӽڵ��Ƿ��Ѿ�������treeNodes��δ����׷�ӵ�treeNodes
    if ismember(childNode, treeNodes(:,1))
        continue
     else
        treeNodes(end+1,:) = [childNode, parentNode];
    end
    
    % �ж��ӽڵ��Ƿ�λ��Ŀ������
    [row_childNode, col_childNode] = ind2sub([rows, cols], childNode);
    [row_goalPos, col_goalPos] = ind2sub([rows, cols], goalPos);
    if abs(row_childNode - row_goalPos) + ...
            abs(col_childNode - col_goalPos) < 2
        break
    end
 
end


%% �ҳ�Ŀ������·��

% ����·��
path_opt = [];
idx = size(treeNodes,1);
while true
    path_opt(end+1) = treeNodes(idx,1);
    parentNode = treeNodes(idx,2);
    if parentNode == startPos
        break;
    else
        idx = find(treeNodes(:,1) == parentNode);
    end    
end

% ·����Ϣ��ӳ��field��
field(treeNodes(:,1)) = 3;
field(path_opt) = 6;
field(startPos) = 4;
field(goalPos) = 5;

%% ��դ��ͼ 
image(1.5,1.5,field);
grid on;
set(gca,'gridline','-','gridcolor','k','linewidth',2,'GridAlpha',0.5);
set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
axis image;