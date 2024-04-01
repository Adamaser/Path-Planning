function influencedChildNnodes = findInfluencedNnodes(field,Nodes,obsNodes)
[rows, cols] = size(field);
influencedChildNnodes = [];         % ��Ӱ����ӽڵ�
influencedParentNnodes = obsNodes;  % ��Ӱ��ĸ��ڵ�
while true
    influencedChildNnode = [];
    for i = 1:length(influencedParentNnodes)
        % ���ÿһ����Ӱ�츸�ڵ㣬�ҵ������ڽ��ڵ�
        % �ж�����ڽ��ڵ��Ƿ��ܵ���Ӱ��
        
        influencedParentNnode = influencedParentNnodes(i);
        childNodes = getNeighborNode(field,[], influencedParentNnode);
        for j = 1:length(childNodes)
            childNode = childNodes(j);
            [childNode_sub(1), childNode_sub(2)] = ind2sub([rows,cols],childNode);
            parentNodes = getNeighborNode(field,[], childNode);
            value = [];
            for k = 1:length(parentNodes)
                parentNode = parentNodes(k);
                [parentNode_sub(1), parentNode_sub(2)] = ind2sub([rows,cols],parentNode);
                cost = norm(parentNode_sub - childNode_sub);
                g = Nodes(parentNode).g;
                value(k) = cost + g;
            end
            
            % ��ýڵ�childNode����С���ڵ�
            [~,idx] = min(value);
            node = parentNodes(idx);
            
            if node == influencedParentNnode
                influencedChildNnode(end+1) = childNode;
            end
        end
    end
    
    influencedChildNnodes = [influencedChildNnodes, influencedChildNnode];
    influencedParentNnodes = influencedChildNnode;
    if isempty(influencedParentNnodes)
        break
    end
    
end

% influencedChildNnodes��ĳЩ�ڵ���ܶ�γ���
% ��unique����ɾ���ظ����ֵĵ�
influencedChildNnodes = unique(influencedChildNnodes);