% ����դ���ͼ�Ļ�����·���滮�㷨
% LPA*�㷨
clc
clear
close all

%% ����դ�񳡾�
% դ������С:����������
rows = 10;
cols = 20;
[field,cmap] = defColorMap(rows, cols);

% ��ʼ���Ŀ���
startPos = 2;
goalPos = rows*cols-2;
field(startPos) = 4;
field(goalPos) = 5;
field1 = field;

%% ��һ�ι滮·��
% ����1��initialize ����ʼ���ڵ���Ϣ�ṹ��
for i = 1:rows*cols
    Nodes(i).node = i;
    Nodes(i).g = inf;
    Nodes(i).rhs = inf;
    Nodes(i).parent = nan;
end
Nodes(startPos).rhs = 0;   % ��Ŀ����rhs=0

% ������ʼ���rhs=0,g=inf,���߲���ȣ�����ӵ�����U��
U(1,1) = startPos;
U(1,2:3) = calculateKey(Nodes(startPos),goalPos, rows, cols);

% ����2��computeShortestPath
[Nodes, U] = computeShortestPath(field1, Nodes, U,rows,cols,startPos,goalPos);

% ��Ŀ��㵹�ƣ����ݸ��ڵ���Ϣ�ҵ�·��
node = goalPos;
path1 = node;
while true
    path1(end+1) = Nodes(node).parent;
    node = Nodes(node).parent;
    if node == startPos
        break
    end
end
path1 = path1(end:-1:1);

% ��·����Ϣ��ӳ��field1��
field1(path1(2:end-1)) = 6;

% ��դ��ͼ��·��
image(1.5,1.5,field1);
grid on;
set(gca,'gridline','-','gridcolor','k','linewidth',2,'GridAlpha',0.5);
set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
axis image;

%% �ڶ��ι滮·�����滮��·�������ϰ���
obsNodes = path1(8:11);
field2 = field;
field2(obsNodes) = 2;

% �ҵ���obsNode��Ϊ���ڵ�������ܵ�Ӱ����ӽڵ�
influencedChildNnodes = findInfluencedNnodes(field,Nodes,obsNodes);

% UpdateEdgeCost
for i = 1:length(obsNodes)
    obsNode = obsNodes(i);
    Nodes(obsNode).rhs = inf;
    Nodes(obsNode).g = inf;
end
[Nodes,U] =  UpdateEdgeCost(influencedChildNnodes,Nodes,U,rows,cols,startPos);

% computeShortestPath
[Nodes, U] = computeShortestPath(field2, Nodes, U,rows,cols,startPos,goalPos);

% ��Ŀ��㵹�ƣ����ݸ��ڵ���Ϣ�ҵ�·��
node = goalPos;
path2 = node;
while true
    path2(end+1) = Nodes(node).parent;
    node = Nodes(node).parent;
    if node == startPos
        break
    end
end
path2 = path2(end:-1:1);

% ���»�դ��ͼ
figure
colormap(cmap);
field2(obsNodes) = 3;
field2(path2) = 7;
field2(startPos) = 4;
field2(goalPos) = 5; 
image(1.5,1.5,field2);
grid on;
set(gca,'gridline','-','gridcolor','k','linewidth',2,'GridAlpha',0.5);
set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
axis image;
