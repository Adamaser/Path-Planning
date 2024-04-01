function childNodes = getNeighborNode(field,closeList, parentNode)
% ѡȡ���ڵ��ܱ�8���ڵ���Ϊ��ѡ�ӽڵ㣬���Ի�����
% �ų������߽�֮��ġ�λ���ϰ����ġ�λ��closeList�е�

[rows, cols] = size(field);
[row_parentNode, col_parentNode] = ind2sub([rows, cols], parentNode);
childNodes = [];

% ��1���ӽڵ�
childNode = [row_parentNode, col_parentNode+1];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end

% ��2���ӽڵ�
childNode = [row_parentNode-1, col_parentNode+1];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end


% ��3���ӽڵ�
childNode = [row_parentNode-1, col_parentNode];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end


% ��4���ӽڵ�
childNode = [row_parentNode-1, col_parentNode-1];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end


% ��5���ӽڵ�
childNode = [row_parentNode, col_parentNode-1];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end


% ��6���ӽڵ�
childNode = [row_parentNode+1, col_parentNode-1];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end


% ��7���ӽڵ�
childNode = [row_parentNode+1, col_parentNode];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end


% ��8���ӽڵ�
childNode = [row_parentNode+1, col_parentNode+1];
if ~(childNode(1) < 1 || childNode(1) > rows ||...
        childNode(2) < 1 || childNode(2) > cols)
    if field(childNode(1), childNode(2)) ~= 2
        childNode_LineIdx = sub2ind([rows, cols], childNode(1), childNode(2));
        if ~ismember(childNode_LineIdx, closeList)
            childNodes(end+1) = childNode_LineIdx;
        end
    end
end



