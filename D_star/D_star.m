% ����դ���ͼ�Ļ�����·���滮�㷨
% D_star�㷨
clc
clear
close all

%% ����ColorMap
rows = 10;
cols = 20;
[field,cmap] = defColorMap(rows, cols);

% ��ʼ���Ŀ���
startPos = 2;
goalPos = rows*cols-2;
field(startPos) = 4;
field(goalPos) = 5;

%% ��һ�׶Σ�����·��

% ��ʼ��Nodes�ṹ�壬����node��t��k��h��parent
field1 = field;
Nodes = struct;
for i = 1:rows*cols
    Nodes(i).node = i;         % �ڵ���������
    Nodes(i).t = 'new';        % �õ��״̬
    Nodes(i).k = inf;          % �õ���Ŀ�����ۼ���С����
    Nodes(i).h = inf;          % �õ���Ŀ��㵱ǰ�������
    Nodes(i).parent = nan;     % ����ڵ�
end

% ��ʼ��Ŀ��ڵ�
Nodes(goalPos).t = 'open';       % �õ��״̬
Nodes(goalPos).k = 0;            % �õ���Ŀ�����ۼ���С����
Nodes(goalPos).h = 0;            % �õ���Ŀ���ľ���
Nodes(goalPos).parent = nan;     % ����ڵ�

% ��goalPos�ŵ�openList
openList = [goalPos, Nodes(goalPos).k];

% ѭ������
while true
    [Nodes, openList, k_old] = process_state(field1, Nodes, openList,goalPos);
    
    % ���ж�����ΪNodes(startPos).t == 'closed'�������˳�����ʱ����A*�㷨��
    % ���ж�����Ϊisempty(openList)�������˳�����ʱ����Dijstra�㷨��
    if isempty(openList)
        break
    end
end

% �ҵ�·��
node = startPos;
path1 = node;
while true
    path1(end+1) = Nodes(node).parent;
    node = Nodes(node).parent;
    if node == goalPos
        break
    end
end

% �ҳ�Ŀ������·��
field1(path1(2:end-1)) = 6;

% ��һ�׶�·����ͼ
image(1.5,1.5,field1);
grid on;
set(gca,'gridline','-','gridcolor','k','linewidth',2,'GridAlpha',0.5);
set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
axis image;

%% �ڶ��׶Σ������ϰ����������·��
% ���ⵯ��ͼ��
figure
colormap(cmap);

% �ϰ�������
obsNode = path1(8:11);

% ���ó�����·��
field2 = field;
field2(obsNode) = 3;
node = startPos;   % �����˵�ǰλ��
path2 = node;
flag = 0;
while node ~= goalPos
    
    % ����field��ɫ
    if flag 
        field2(path2(flag:end)) = 7;
    else
        field2(path2) = 6;
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
    
    % �����ڵ�parentNode��Ϊ�ϰ���������ִ��process_state������
    parentNode = Nodes(node).parent;
    if field2(parentNode) == 2 || field2(parentNode) == 3
        % �����޸ĸ��ڵ��hֵ
        Nodes(parentNode).h = inf;
        
        % ��node��ʶΪclosed�����޸���hֵ������ӵ�openList
        if isequal(Nodes(node).t, 'closed')
            Nodes(node).h = inf;
            [Nodes,openList] = insert(Nodes,openList,node,Nodes(node).h);
        end
        
        while true
            [Nodes, openList, k_min] = process_state(field2, Nodes, openList,goalPos);
            if k_min >= Nodes(node).h
                break
            end
        end
        
        % ��ǵ�ǰ·��path2�����ݳ��ȣ�������һ����ɫ��ͼ
        flag = length(path2);
    end
    
    node = Nodes(node).parent;
    path2(end+1) = node;
end
