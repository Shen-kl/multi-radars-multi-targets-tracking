function [state ,step] = step5(state)
%     Step5：Construct a series of alternating primed and starred zeros as follows.
%     Let Z0 represent the uncovered primed zero found in Step 4.
%     Let Z1 denote the starred zero in the column of Z0 (if any).
%     Let Z2 denote the primed zero in the row of Z1 (there will always be one).
%     Continue until the series terminates at a primed zero that has no starred
%     zero in its column. Unstar each starred zero of the series, star each
%     primed zero of the series, erase all primes and uncover every line in the
%     matrix. Return to Step 3
% 
% 	构造如下一系列交替的填色和加星号的零：
%      令Z0代表在步骤4中发现的未覆盖的准备好的零 0'。
%      令Z1表示Z0列中的星号零 0*（如果有的话）。
%      令Z2表示Z1行中的准备好的零 0'（始终为1个）。
%      继续直到0'所在列没有星标0*，终止该序列。取消对每个已加星标的零的星标，对系列中的每个0'加星标，去除所有的'和覆盖线。 返回步骤3。
%     """
    count = 1;
    path = state.path;
    path(count, 1) = state.Z0_r;
    path(count, 2) = state.Z0_c;

    while (1)
%         # Find the first starred element in the col defined by
%         # the path.
        [M,row] = max(state.marked(:, path(count, 2)) == 1);
        if  state.marked(row, path(count, 2)) ~= 1
%             # Could not find one
            break;
        else
            count = count + 1 ;
            path(count, 1) = row;
            path(count, 2) = path(count - 1, 2);
        end
%         # Find the first prime element in the row defined by the
%         # first path step
        [M,col] = max(state.marked(path(count, 1),:) == 2);
        if state.marked(row, col) ~= 2
            col = size(state.marked,1) ;
        end
        count = count + 1;
        path(count, 1) = path(count - 1, 1);
        path(count, 2) = col;        
    end
%     # Convert paths
    for i =1 : count
        if state.marked(path(i, 1), path(i, 2)) == 1
            state.marked(path(i, 1), path(i, 2)) = 0;
        else
            state.marked(path(i, 1), path(i, 2)) = 1;
        end
    end

    state  = clear_covers(state);
%     # Erase all prime markings
    state.marked(state.marked == 2) = 0;
    state.path = path;
    step ='step3';
    return;
    

end

