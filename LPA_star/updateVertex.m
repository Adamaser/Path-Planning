function [Nodes,U] = updateVertex(field,Nodes,U,node,rows,cols,startPos,goalPos)
if node ~= startPos
    Nodes(node).rhs = inf;
    [node_sub(1), node_sub(2)] = ind2sub([rows,cols],node);
    
    % �ҵ���ǰ�ڵ����Χ�����ڽ��ڵ�
    neigborNodes = getNeighborNode(field,[], node);
    
    % ���������ڽ��ڵ��g(s')+C(s',u)
    value = [];
    for i = 1:length(neigborNodes)
        neighborNode = neigborNodes(i);
        [neighborNode_sub(1), neighborNode_sub(2)] = ind2sub([rows,cols],neighborNode);
        cost = norm(neighborNode_sub - node_sub);
        value(i) = Nodes(neighborNode).g + cost; 
    end
    
    % ��ȡ��Χg(s')+C(s',u)��ֵ��С���ڽ��ڵ㣬��ֵ��rhs(u)�������¸��ڵ�
     [Nodes(node).rhs, idx] = min(value);
     Nodes(node).parent = neigborNodes(idx); 
end   

% ���node�Ѿ�������U���Ƴ�
[in,idx] = ismember(node, U(:,1));
if in
    U(idx,:) = [];
end

% ���node��rhs��g����ȣ�����ӵ�U��
if Nodes(node).rhs ~= Nodes(node).g
    U(end+1,1) = node;
    U(end,2:3) = calculateKey(Nodes(node),goalPos, rows, cols);
end
