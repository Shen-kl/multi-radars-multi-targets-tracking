function [state ,step] = step1(state)
%     Step 1: For each row of the matrix, find the smallest element and
%     subtract it from every element in its row.    
%     减去每一行的最小值

    state.C = state.C - min(state.C,[],2);
    
%     Step 2: Find a zero (Z) in the resulting matrix. If there is no
%     starred zero in its row or column, star Z. Repeat for each element
%     in the matrix.    
%     如果一行或列中没有星标的0，则标记0*

    [row ,col] =find(state.C == 0);
    for i = 1:size(row,1)
        if state.col_uncovered(col(i)) && state.row_uncovered(row(i))
            state.marked(row(i),col(i)) = 1;
            state.col_uncovered(col(i)) = false;
            state.row_uncovered(row(i)) = false;      
        end
    end
    state  = clear_covers(state);
    
    step ='step3';
    return ;

end

