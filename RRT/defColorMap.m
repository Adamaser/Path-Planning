function [field,cmap] = defColorMap(rows, cols)
cmap = [1 1 1; ...       % 1-��ɫ-�յ�
    0 0 0; ...           % 2-��ɫ-��̬�ϰ�
    1 0 0; ...           % 3-��ɫ-�Ѿ��������ĵ�
    1 1 0;...            % 4-��ɫ-��ʼ�� 
    1 0 1;...            % 5-Ʒ��-Ŀ���
    0 1 0; ...           % 6-��ɫ-��Ŀ���Ĺ滮·��   
    0 1 1];              % 7-��ɫ-��̬�滮��·��

% ������ɫMAPͼ
colormap(cmap);

% ����դ���ͼȫ�򣬲���ʼ���հ�����
field = ones(rows, cols);

% �ϰ�������
obsRate = 0.3;
obsNum = floor(rows*cols*obsRate);
obsIndex = randi([1,rows*cols],obsNum,1);
field(obsIndex) = 2;
