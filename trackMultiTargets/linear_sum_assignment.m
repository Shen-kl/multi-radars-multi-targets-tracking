function [row ,loc] = linear_sum_assignment(cost_matrix)
%函数功能：使用匈牙利算法解决最优指派问题
%参数说明 cost_matrix 代价矩阵
    if size(cost_matrix,2) < size(cost_matrix,1)
        cost_matrix =cost_matrix'; %保证行数小于等于列数
        transposed = true;
    else
        transposed = false;
    end
    
    state = Hungry(cost_matrix);
    
    step ='step1';
    
    while ~strcmp(step,'None')
        switch step
            case 'step1'
               [state ,step]= step1(state);
            case 'step3'
               [state ,step]= step3(state);
            case 'step4'
               [state ,step]= step4(state);
            case 'step5'
               [state ,step]= step5(state);
            case 'step6'
               [state ,step]= step6(state);
        end
    end    
    if transposed
        marked = state.marked';
    else
        marked = state.marked;
    end
    
    [row, loc] = find(marked == 1);
    [row Q]=sort(row);
    loc=loc(Q);
end

