function [Nodes, U] = computeShortestPath(field, Nodes, U, rows,cols,startPos,goalPos)

% �������A*�㷨������closeList���ϣ������ų������ڽڵ�
closeList = [];
while true
    
    % ��U������k1����k2����
    U = sortrows(U,[2 3]);
    
    % ����ҵ���Ŀ��㣬����U(1,2)>calculateKey(goalPos),�˳�ѭ��
    if U(1,2) > calculateKey(Nodes(U(1,1)),goalPos, rows, cols)
        break
    end
    if ~isinf(Nodes(goalPos).rhs) && Nodes(goalPos).rhs == Nodes(goalPos).g
        break
    end
    
    
    node = U(1,1);
    closeList(end+1) = node;
    if Nodes(node).g > Nodes(node).rhs   % �ֲ���һ��
        
        Nodes(node).g =   Nodes(node).rhs;
        [Nodes,U] = updateVertex(field,Nodes,U,node,rows,cols,startPos,goalPos);
        
        % �ҵ���ǰ�ڵ�������ڽ��ڵ㣬����λ��closeList�еĽڵ�
        neighborNodes = getNeighborNode(field,closeList, node);
        
        % ���������ӽڵ��rhs
        for i = 1:length(neighborNodes)
            neighborNode = neighborNodes(i);
            [Nodes,U] = updateVertex(field,Nodes,U,neighborNode,rows,cols,startPos,goalPos);
        end  
        
    else  % �ֲ�һ�»�ֲ�Ƿһ��
        Nodes(node).g = inf;
        [Nodes,U] = updateVertex(field,Nodes,U,node,rows,cols,startPos,goalPos);
        
        % �ҵ���ǰ�ڵ�������ڽ��ӽڵ㣬����λ��closeList�еĽڵ�
        neighborNodes = getNeighborNode(field,closeList, node);
        
        % ���������ӽڵ��rhs
        for i = 1:length(neighborNodes)
            neighborNode = neighborNodes(i);
            [Nodes,U] = updateVertex(field,Nodes,U,neighborNode,rows,cols,startPos,goalPos);
        end
    end
 
end