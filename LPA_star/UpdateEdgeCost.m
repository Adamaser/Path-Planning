function [Nodes,U] = UpdateEdgeCost(influencedNnodes,Nodes,U,rows,cols,startPos )

for i = 1:length(influencedNnodes)
    node = influencedNnodes(i);
    Nodes(node).rhs = inf;
    
    % ����Nodes(node).rhs ~= Nodes(node).h�����뵽U��
    U(end+1,1) = node;
    U(end,2:3) = calculateKey(Nodes(node),startPos, rows, cols);
end