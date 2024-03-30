function [Nodes, openList, k_old] = process_state(field, Nodes, openList,goalPos)

% ���openList��kֵ��С�Ľڵ�X��k_old
[k_old,idx] = min(openList(:,2));
X = openList(idx,1);

% ��openList���Ƴ�X�����޸���״̬Ϊclosed
openList(idx,:) = [];
Nodes(X).t = 'closed';

% X���ڽڵ�
neighborNodes = getNeighborNode(field, X);

%% ����X�ĸ��ڵ㼰�����Ϣ
% �ж�k_old �Ƿ�С�� Nodes(X).h,���Ǳ����ýڵ��Ѿ��ܵ��ϰ���Ӱ��
% ��ô���ж�X���ڽڵ㣬���Ƿ��ܹ���ĳ���ڽڵ���Ϊ���ڵ㣬ʹNodes(X).h��С

if k_old < Nodes(X).h
    
    % ����X���ڽڵ�Y
    for i = 1:size(neighborNodes,1)
        Y = neighborNodes(i,1);
        cost_X_Y = neighborNodes(i,2);
        
        % ���Y���hֵû������������X��hֵ��ͨ��Y��ø�С,��ô���޸�X�ĸ��ڵ�ΪY��������h��ֵ��
        % ͬʱ�ж�Nodes(Y).h <= k_old������Y�����Ƿ��յ��ϰ�Ӱ�쵼��Nodes(Y).h > k_old
        % ��Nodes(Y).h > k_old����ô����Y��Ϊ���ڵ��û��������
        if Nodes(Y).h <= k_old && Nodes(X).h > Nodes(Y).h + cost_X_Y
            Nodes(X).parent = Y;
            Nodes(X).h = Nodes(Y).h + cost_X_Y;
            % ע�⣬�˴�������Nodes(X).h��ֻ���� Nodes(X).h��һ����С�ˣ�
            % ����k_old˭��˭С����δ��֪����Ҫ��һ�ڼ����ж�
        end
    end
end


%% ��һ��������X����Ϣ����һ���жϣ�������Y�ĸ��ڵ㼰�����Ϣ

if k_old == Nodes(X).h   
    % ��k_old = Nodes(X).h��Lower̬�����������ֿ��ܣ�
    % 1�����ڵ�һ������Ľ׶Σ�
    % 2���ýڵ�X��û���ܵ��ϰ�Ӱ�죻
    % 3���ܵ����ϰ���Ӱ�죬��������һ���Ѿ�������X��parent

    % ����X���ڽڵ�Y
    for i = 1:size(neighborNodes,1)
        Y = neighborNodes(i,1);
        cost_X_Y = neighborNodes(i,2);
        if  isequal(Nodes(Y).t, 'new')...
                ||  Nodes(Y).parent == X &&  Nodes(Y).h ~= Nodes(X).h + cost_X_Y...
                ||  Nodes(Y).parent ~= X &&  Nodes(Y).h > Nodes(X).h + cost_X_Y && Y ~= goalPos
            % �����1�������ڽڵ�Y��δ����openList����ô����X��Ϊ���ڵ㣻
            % �����2��������ȻY�ĸ��ڵ���X������ Nodes(Y).hȴ��Nodes(X).h + cost_X_Y������ˣ�
            % ����Y�ĸ��ڵ�Nodes(X).h �й����£������������ϰ�����ģ�
            % �����3������Y����ͨ����X��Ϊ���ڵ㣬ʹ��Nodes(Y).h��С
            % �����������Ӧ�ý�X��ΪY�ĸ��ڵ㣬����Y�Ƶ�openList���ٽ�һ������
            Nodes(Y).parent = X;
            h_new = Nodes(X).h + cost_X_Y;
            [Nodes,openList] = insert(Nodes,openList,Y,h_new);
        end

    end
   

   
else 
    % ��k_old ~= Nodes(X).h,Raise̬
    % ˵���ڵ�X�ܵ���Ӱ�죬����������
    
    for i = 1:size(neighborNodes,1)
        Y = neighborNodes(i,1);
        cost_X_Y = neighborNodes(i,2);
        if  isequal(Nodes(Y).t, 'new')||...
                Nodes(Y).parent == X && Nodes(Y).h ~= Nodes(X).h + cost_X_Y
            % �����1�������ڽڵ�Y��δ����openList����ô����X��Ϊ���ڵ㣻
            % �����2��������ȻY�ĸ��ڵ���X������ Nodes(Y).hȴ��Nodes(X).h + cost_X_Y������ˣ�
            % �����������Ӧ�ý�X��ΪY�ĸ��ڵ㣬����Y�Ƶ�openList���ٽ�һ������
            Nodes(Y).parent = X;
            h_new = Nodes(X).h + cost_X_Y;
            [Nodes,openList] = insert(Nodes,openList,Y,h_new);
        else

            if  Nodes(Y).parent ~= X && Nodes(Y).h > Nodes(X).h + cost_X_Y
                % �����㣬����Y����ͨ����X��Ϊ���ڵ㣬ʹ��Nodes(Y).h��С                
                % ����Ŀǰ���X���� k_old ~= Nodes(X).h����Ҫ��X׷�ӵ�openList������һ��ѭ�����������ͽ�X��ΪY�ĸ��ڵ㡣
                [Nodes,openList] = insert(Nodes,openList,X,Nodes(X).h);
            elseif  Nodes(Y).parent ~= X && Nodes(X).h > Nodes(Y).h + cost_X_Y...
                    &&  isequal(Nodes(Y).t, 'closed') && Nodes(Y).h > k_old
               % ���1������Y�ĸ��ڵ㲻��X��������Y��ΪX���ڵ㣬 Nodes(X).h��С��
               % ���2����������Y�Ѿ���open���Ƴ���
               % ���3����ǰ��openListȡ������Сֵk_old��Ȼ��h(Y)С�������Ѿ����Ƴ�open���Y�ܵ����ϰ�Ӱ�쵼��hֵ����
               % �����������Ҫ���½�Y����openList�У�������һ�ֿ��졣              
                [Nodes,openList] = insert(Nodes,openList,Y,Nodes(Y).h);
            end
        end
    end
end

end
