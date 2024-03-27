function flag = judgeObs(field, parentNode, childNode)

flag = 0;
[rows, cols] = size(field);

% �ж��ӽڵ��Ƿ����ϰ�����
obsIdx = find(field == 2);
if ismember(childNode, obsIdx)
    flag = 1;
    return
end

% �жϸ��ڵ����ӽڵ�������Ƿ����ϰ���
[parentNode(1), parentNode(2)] = ind2sub([rows, cols], parentNode);
[childNode(1), childNode(2)] = ind2sub([rows, cols], childNode);

P2 = parentNode;
P1 = childNode;
row_min = min([P1(1), P2(1)]);
row_max = max([P1(1), P2(1)]);
col_min = min([P1(2), P2(2)]);
col_max = max([P1(2), P2(2)]);
for row = row_min:row_max
    for col = col_min:col_max
        if field(row, col) == 2
            P = [row, col];
            
            % ֱ�Ӽ����ϰ���ڵ��P1��P2���ɵ����ߵľ���
            d = abs(det([P2-P1;P-P1]))/norm(P2-P1);
            if d < 0.5
                flag = 1;
                return
            end
        end
    end
end
            
