function [Nodes, U] = computeShortestPath(field, Nodes, U, km,rows,cols,startPos,goalPos)

while true
    
    % ��U������k1����k2����
    U = sortrows(U,[2 3]);
    u = U(1,1);
    k_old = U(1,2);
    k_u = calculateKey(Nodes(u),startPos, km, rows, cols);
    U(1,:) = [];
    
    % ����ҵ��˻����˵�ǰλ�õ㣬����U(1,2)>calculateKey(startPos),�˳�
    k_start = calculateKey(Nodes(startPos),startPos, km, rows, cols);
    if k_old > k_start(1)
        break
    end
    if ~isinf(Nodes(startPos).rhs) && Nodes(startPos).rhs == Nodes(startPos).g
        break
    end
    
    if k_old < k_u(1)
        % �����㣬�����ڵ�u��kֵ�����޸ģ����ܵ��˻�����Ӱ�죨��ͻ���ϰ��
        % ��Ӧ�����²��뵽U�����У�����һ���Ŀ���
        U(end+1,:) = [u,k_u];
        
    elseif Nodes(u).g > Nodes(u).rhs
        % �����㣬�ֲ���һ��״̬�������ڵ�u���µĽݾ�
        % ��ô�Ϳ���u���ڽ��ڵ㣬�ж���u��Ϊ���ڵ��Ƿ�·�����ţ�������
        Nodes(u).g = Nodes(u).rhs;
        neighborNodes = getNeighborNode(field, u);
        for i = 1:length(neighborNodes)
            neighborNode = neighborNodes(i);
            [Nodes,U] = updateVertex(field,Nodes,U,neighborNode, km,rows,cols,startPos,goalPos);
        end
    else
        % �������������㣬���������ϰ���ֲ�Ƿһ��״̬
         Nodes(u).g = inf;
         
         % ����u���ڽ��ڵ�
         neighborNodes = getNeighborNode(field, u);
         influencedNnodes = [neighborNodes,u];
         for i = 1:length(influencedNnodes)
             s = influencedNnodes(i);
             [Nodes,U] = updateVertex(field,Nodes,U,s, km,rows,cols,startPos,goalPos);
         end
    end
end
