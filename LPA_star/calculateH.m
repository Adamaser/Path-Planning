function h = calculateH(pos_now, pos_goal, rows, cols)
% ����Ľڵ㵽Ŀ���Ĺ���ֵh���㷽ʽ��A*�㷨���岻ͬ
% A*�㷨��ֱ���ñ��ڵ��Ŀ����h = �в� + �в�
% ��ϵ�пγ̿��ǵ���������ԣ������������һ�ּ��㷽ʽ
% ������ֵ����Ϊh = ���б���˶����� + ���������˶�����  

[pos_now_sub(1),pos_now_sub(2)] = ind2sub([rows, cols], pos_now);
[pos_goal_sub(1),pos_goal_sub(2)] = ind2sub([rows, cols], pos_goal);
diff_row = abs(pos_now_sub(1) - pos_goal_sub(1));
diff_col = abs(pos_now_sub(2) - pos_goal_sub(2));
diff_min = min(diff_row,diff_col);
diff_max = max(diff_row,diff_col);
h = sqrt(2)*diff_min + (diff_max - diff_min);
end
