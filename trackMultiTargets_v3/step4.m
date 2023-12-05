function [state ,step] = step4(state)

%     Step4：Find a noncovered zero and prime it. If there is no starred zero
%     in the row containing this primed zero, Go to Step 5. Otherwise,
%     cover this row and uncover the column containing the starred
%     zero. Continue in this manner until there are no uncovered zeros
%     left. Save the smallest uncovered value and Go to Step 6.
%     
%     找到一个未覆盖的零并将其准备好。 如果准备好的零所在行中没有加星号的零，
%     请转到步骤5。否则，覆盖该行并找出包含加注星号的零的列。 继续以这种方式
%     进行操作，直到没有剩余的零为止。保存最小的发现值，然后转到步骤6。
%     """
%     # We convert to int as numpy operations are faster on int
    C = (state.C == 0);
    covered_C = C .* state.row_uncovered;
    covered_C = covered_C .*  state.col_uncovered';
    n = size(state.C,1);
    m = size(state.C,2);

    while (1)
%         # Find an uncovered zero
        [M,I] = max(covered_C(:));
        [row, col] = ind2sub(size(covered_C),I);
        if covered_C(row, col) == 0
             step ='step6';
             return ;
        else
            state.marked(row, col) = 2;
%             # Find the first starred element in the row
            [~,star_col ]= max(state.marked(row,:) == 1);
            if state.marked(row, star_col) ~= 1
%                 # Could not find one
                state.Z0_r = row;
                state.Z0_c = col;
                step ='step5';
                return ;
            else
                col = star_col;
                state.row_uncovered(row) = false;
                state.col_uncovered(col) = true;
                covered_C(:, col) = C(:, col) .* state.row_uncovered;
                covered_C(row,:) = 0;
            end
        end
    end

end

