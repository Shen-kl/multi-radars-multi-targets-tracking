function [state ,step] = step6(state)
%     Step 6: Add the value found in Step 4 to every element of each covered row,
%     and subtract it from every element of each uncovered column.
%     Return to Step 4 without altering any stars, primes, or covered lines.
% 	
% 	将在第4步中找到的值添加到每个覆盖行的每个元素中，
%     并将其从每个未覆盖列的每个元素中减去。
%     返回第4步，而不更改任何星号，或遮盖线。
%     """
%     # the smallest uncovered value in the matrix
    if any(state.row_uncovered) && any(state.col_uncovered)
        [M,I]=find(state.row_uncovered  == 1);
        [M1,I1]=find(state.col_uncovered  == 1);
        minval = min(min(state.C(M,M1)));
        [M2,I2]=find(state.row_uncovered  == 0);
        if ~isempty(M2)
            state.C(M2,:) =state.C(M2,:) + minval;
        end
        state.C(:, M1) =state.C(:, M1) - minval;
        
    end
   
    step ='step4';
    return;

end

