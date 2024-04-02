% ����դ���ͼ�Ļ�����·���滮�㷨
% ��8�ڣ�D* lite�㷨
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
Nodes(goalPos).rhs = 0;   % ��Ŀ����rhs=0

% ��һ��·���滮ʱ��km(key����ֵ)Ϊ0
km = 0;

% ������ʼ���rhs=0,g=inf,���߲���ȣ�����ӵ�����U��
U(1,1) = goalPos;
U(1,2:3) = calculateKey(Nodes(goalPos),startPos, km, rows, cols);

% ����2��computeShortestPath
[Nodes, U] = computeShortestPath(field1, Nodes, U, km,rows,cols,startPos,goalPos);

% ����ʼ�㿪ʼ�����ݸ��ڵ���Ϣ�ҵ�·��
node = startPos;
path1 = node;
while true
    path1(end+1) = Nodes(node).parent;
    node = Nodes(node).parent;
    if node == goalPos
        break
    end
end

% ��·����Ϣ��ӳ��field1��
field1(path1(2:end-1)) = 6;

% ��դ��ͼ��·��
image(1.5,1.5,field1);
grid on;
set(gca,'gridline','-','gridcolor','k','linewidth',2,'GridAlpha',0.5);
set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
axis image;

%% ����5������������·���˶����滮·��ǰ�������ϰ���
% ��field��path��ֵ
field2 = field;
path2 = path1;

% �����ϰ���ڵ�λ�á�s_start��s_last
obsRange = 8:11;               % �ϰ���λ�ڹ滮·���ķ�Χ
obsNodes = path2(obsRange);    % �ϰ���դ��
obsPreviewRange = 2;           % �ϰ���Ԥ�����
s_start = path2(1);
s_last = s_start;

% ��ͼ��
figure
colormap(cmap);
flag = 0;        % �����ж�դ�񳡾��Ƿ������ϰ�����µı�ʶ
step = 1;        % �����˵��˶�����

% �����˿�ʼ�ƶ�
while s_start ~= goalPos
    
    % �������˻�δ�ƶ���Ԥ���ϰ����Ԥ�������һ��դ��
    % ������˰���path1�����˶�
    % ���flag=1�������Ѿ��������ϰ�����£�path��֮���£�������path2�˶�
    if step ~= obsRange(1) - obsPreviewRange
        s_start = path2(step);
        
        % ����field��ɫ
        field2(obsNodes) = 3;
        field2(path2(1:step)) = 6;
        if flag
            field2(path2(obsRange(1) - obsPreviewRange+1:step)) = 7;
        end
        
        field2(startPos) = 4;
        field2(goalPos) = 5;
        
        % ��դ��ͼ
        image(1.5,1.5,field2);
        grid on;
        set(gca,'gridline','-','gridcolor','k','linewidth',2,'GridAlpha',0.5);
        set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
        axis image;
        pause(0.3)
       
    % ���������ƶ���Ԥ���ϰ����Ԥ�������һ��դ��
    % ����벽��5�����������ܵ�Ӱ��Ľڵ���ۣ�������Ѱ·
    else 
        
        % ����������ڵ�����䣬 ����ͣ2�룬ģ������Ѱ·
        disp('�����ϰ������Ѱ·...')
        pause(2)     
        
        % ����field2��s_start��km
        field2(obsNodes) = 2;
        s_start = path2(step);
        km = km +  calculateH(s_start, s_last, rows, cols);
        s_last = s_start; 
        
        % �ҵ���obsNodes��Ϊ���ڵ�������ܵ�Ӱ����ӽڵ�
        influencedChildNnodes = findInfluencedNnodes(field,Nodes,obsNodes);
     
        % ���������ܵ�Ӱ����ӽڵ�Ĵ���ֵ
        for i = 1:length(obsNodes)
            obsNode = obsNodes(i);
            Nodes(obsNode).rhs = inf;
            Nodes(obsNode).g = inf;
        end
        [Nodes,U] =  UpdateEdgeCost(influencedChildNnodes,Nodes,U,km,rows,cols,s_start);

        % computeShortestPath
        [Nodes, U] = computeShortestPath(field2, Nodes, U, km,rows,cols,s_start,goalPos);
               
        % �ӻ����˵�ǰλ�ÿ�ʼ�����ݸ��ڵ���Ϣ�ҵ�·��
        node = s_start;
        path2(obsRange(1) - obsPreviewRange + 1:end) = [];
        while true
            path2(end+1) = Nodes(node).parent;
            node = Nodes(node).parent;
            if node == goalPos
                break
            end
        end
             
        flag = 1;
       
    end
    step = step + 1;   
end
