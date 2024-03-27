function childNode = getChildNode(field, parentNode, samplePoint)
% ��������������Ϊ2��դ��ѡȡ���ڵ��ܱ�16���ڵ���Ϊ��ѡ�ӽڵ�
% ��������������븸�ڵ�ĽǶȣ�ȷ���������ӽڵ�


[rows, cols] = size(field);
[row_samplePoint, col_samplePoint] = ind2sub([rows, cols], samplePoint);
[row_parentNode, col_parentNode] = ind2sub([rows, cols], parentNode);


% ����16�������������
% ע�⣬Ϊ������������x/y����ƥ�䣬�Ӹ��ڵ���½ڵ���ʱ�뿪ʼ���壬���α��
childNode_set = [ row_parentNode+2, col_parentNode;
    row_parentNode+2, col_parentNode+1;
    row_parentNode+2, col_parentNode+2;
    row_parentNode+1, col_parentNode+2;
    row_parentNode, col_parentNode+2; 
    row_parentNode-1, col_parentNode+2;
    row_parentNode-2, col_parentNode+2;
    row_parentNode-2, col_parentNode+1;
    row_parentNode-2, col_parentNode;
    row_parentNode-2, col_parentNode-1;
    row_parentNode-2, col_parentNode-2;
    row_parentNode-1, col_parentNode-2;
    row_parentNode,   col_parentNode-2;
    row_parentNode+1, col_parentNode-2;
    row_parentNode+2, col_parentNode-2;
    row_parentNode+2, col_parentNode-1];

% ����16���ӽڵ�ĽǶȷ�Χ�����͵�ǰ�����ĽǶȷ�Χ
theta_set = linspace(0,2*pi,16);
theta = atan2((col_samplePoint - col_parentNode), ...
    (row_samplePoint - row_parentNode));

% ��thetaλ�ڵ��������ޣ�����2*pi
if theta < 0
    theta = theta + 2*pi;
end

% ������Χ��16���㣬�жϽǶ�λ����һ����Χ
for i = 1:15
    if theta >= theta_set(i) && theta < theta_set(i+1)
        childNodeIdx = i;
        break
    end
end

% ѡ�е��ӽڵ�
childNode = childNode_set(childNodeIdx,:);


