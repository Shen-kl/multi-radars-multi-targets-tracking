function [state ,step] = step3(state)
%     Step3：Cover each column containing a starred zero. If n columns are covered,
%     the starred zeros describe a complete set of unique assignments.
%     In this case, Go to DONE, otherwise, Go to Step 4.
%     
%     覆盖每列包含加星号的零。如果覆盖了n列，加星号的零表示完整的唯一结果集。

    marked = (state.marked == 1);
    [row,col]=find(marked == true);
    state.col_uncovered(col) = false;

    if sum(sum(marked,1)) < size(state.C,1)
        step ='step4';
        return ;
    end
    
    step ='None';
    return ;
end

