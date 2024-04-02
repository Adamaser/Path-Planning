function [Nodes,U] = updateVertex(field,Nodes,U,u, km,rows,cols,s_start,goalPos)
if u ~= goalPos
    Nodes(u).rhs = inf;
    [node_sub(1), node_sub(2)] = ind2sub([rows,cols],u);
    
    % �ҵ���ǰ�ڵ����Χ�����ڽ��ڵ�
    neigborNodes = getNeighborNode(field, u);
    
    % ���������ڽ��ڵ��g(s')+C(s',u)
    value = [];
    for i = 1:length(neigborNodes)
        neighborNode = neigborNodes(i);
        [neighborNode_sub(1), neighborNode_sub(2)] = ind2sub([rows,cols],neighborNode);
        cost = norm(neighborNode_sub - node_sub);
        value(end+1) = Nodes(neighborNode).g + cost;
    end
    
    % ��ȡ��Χg(s')+C(s',u)��ֵ��С���ڽ��ڵ㣬��ֵ��rhs(u)�������¸��ڵ�
    [Nodes(u).rhs, idx] = min(value);
    Nodes(u).parent = neigborNodes(idx);
end

% ���node�Ѿ�������queue���Ƴ�
[in,idx] = ismember(u, U(:,1));
if in
    U(idx,:) = [];
end

% ���node��rhs��g����ȣ�����ӵ�queue��
if Nodes(u).rhs ~= Nodes(u).g
    U(end+1,1) = u;
    U(end,2:3) = calculateKey(Nodes(u),s_start, km, rows, cols);
end
