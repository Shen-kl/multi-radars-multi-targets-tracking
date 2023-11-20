% Author: Shen Kailun
% Date  : 2023-11-17
function [fusion] = track_process_GNN_fusion(fusion, frame_index, radar_index)
    %时间配准 航迹预测
    if frame_index ~= fusion.system_frame_index
        for track_fusion_index = 1 : fusion.track_fusion_num
            [X_predict, P_predict] = fusion.tracker.KalmanPredict_specifiedT(...
                fusion.track_fusion_set(track_fusion_index).X(:,end),...
                fusion.track_fusion_set(track_fusion_index).P,...
                fusion.T * (frame_index - fusion.system_frame_index));
            fusion.track_fusion_set(track_fusion_index).X = [...
                fusion.track_fusion_set(track_fusion_index).X X_predict];
            fusion.track_fusion_set(track_fusion_index).P = P_predict;
        end
        fusion.system_frame_index = frame_index;
    end
    if isempty(fusion.radar_track_set)
       %若分站航迹为空
       %所有系统航迹 对应分站扣分
        for track_fusion_index = 1 : fusion.track_fusion_num
            fusion.track_fusion_set(track_fusion_index).track_quality(radar_index) = ...
                fusion.track_fusion_set(track_fusion_index).track_quality(radar_index) - 3;
        end        
    else
        %统计航迹类型
        type_0_set = [];
        type_1_set = [];
        type_2_set = [];
        for track_fusion_index = 1 : fusion.track_fusion_num
            switch fusion.track_fusion_set(track_fusion_index).track_type
                case 0
                    type_0_set = [type_0_set track_fusion_index];
                case 1
                    type_1_set = [type_1_set track_fusion_index];
                case 2
                    type_2_set = [type_2_set track_fusion_index];
            end
        end        
        %优先处理类型为2的航迹
        if ~isempty(type_2_set)
            for type_2_index = 1 : size(type_2_set,2)
                %确认当前帧 雷达是否检测到该目标
                [loc, value] = fusion.find_track(fusion.track_fusion_set(type_2_set(type_2_index)).connection_table(radar_index));
                if value == 0
                    %若不存在 则认为目标消失 变为孤立航迹
                    fusion.track_fusion_set(type_2_set(type_2_index)).connection_status = 1;
                    fusion.track_fusion_set(type_2_set(type_2_index)).connection_table(radar_index) = -1;
                    fusion.track_fusion_set(type_2_set(type_2_index)).track_quality(radar_index) = -1;
                else
                    %确认是否落入大小波门
                    fusion.track_fusion_set(type_2_set(type_2_index)).connection_status = 1;
                    fusion.radar_track_set(loc).connection_status = 1;
                    
                    [~,flag] = fusion.Related_gate_track2track(type_2_set(type_2_index),loc,fusion.Chi_large);
                    if flag == 0
                        %没有落入大波门
                        fusion.track_fusion_set(type_2_set(type_2_index)).track_quality(radar_index) = ...
                            fusion.track_fusion_set(type_2_set(type_2_index)).track_quality(radar_index) - 3;
                    else
                        %落入大波门 判断小波门
                        [~,flag1] = fusion.Related_gate_track2track(type_2_set(type_2_index),loc,fusion.Chi_small);
                        if flag1 == 0
                            %没有落入小波门
                            fusion.track_fusion_set(type_2_set(type_2_index)).track_quality(radar_index) = ...
                                fusion.track_fusion_set(type_2_set(type_2_index)).track_quality(radar_index) + 2;                            
                        else
                            %落入小波门
                            fusion.track_fusion_set(type_2_set(type_2_index)).track_quality(radar_index) = ...
                                fusion.track_fusion_set(type_2_set(type_2_index)).track_quality(radar_index) + 3;                            
                        end
                    end
                    
                    %融合
                    [fusion_X, fusion_P] = fusion.convexCombinationFusion(fusion.track_fusion_set(type_2_set(type_2_index)).X(:,end),...
                        fusion.track_fusion_set(type_2_set(type_2_index)).P,...
                        fusion.radar_track_set(loc).X, fusion.radar_track_set(loc).P);
                    fusion.track_fusion_set(type_2_set(type_2_index)).X(:,end) = fusion_X;
                    fusion.track_fusion_set(type_2_set(type_2_index)).P = fusion_P;
                end
            end
        end

        switch radar_index
            case 1
                %当前是雷达1 的数据
                %优先处理类型为0的航迹
                if ~isempty(type_0_set)
                    for type_0_index = 1 : size(type_0_set,2)
                        %确认当前帧 雷达是否检测到该目标
                        [loc, value] = fusion.find_track(fusion.track_fusion_set(type_0_set(type_0_index)).connection_table(radar_index));
                        if value == 0
                            %若不存在 则认为目标消失 变为孤立航迹
                            fusion.track_fusion_set(type_0_set(type_0_index)).connection_status = 1;
                            fusion.track_fusion_set(type_0_set(type_0_index)).connection_table(radar_index) = -1;
                            fusion.track_fusion_set(type_0_set(type_0_index)).track_quality(radar_index) = -1;
                        else
                            %确认是否落入大小波门
                            fusion.track_fusion_set(type_0_set(type_0_index)).connection_status = 1;
                            fusion.radar_track_set(loc).connection_status = 1;

                            [~,flag] = fusion.Related_gate_track2track(type_0_set(type_0_index),loc,fusion.Chi_large);
                            if flag == 0
                                %没有落入大波门
                                fusion.track_fusion_set(type_0_set(type_0_index)).track_quality(radar_index) = ...
                                    fusion.track_fusion_set(type_0_set(type_0_index)).track_quality(radar_index) - 3;
                            else
                                %落入大波门 判断小波门
                                [~,flag1] = fusion.Related_gate_track2track(type_0_set(type_0_index),loc,fusion.Chi_small);
                                if flag1 == 0
                                    %没有落入小波门
                                    fusion.track_fusion_set(type_0_set(type_0_index)).track_quality(radar_index) = ...
                                        fusion.track_fusion_set(type_0_set(type_0_index)).track_quality(radar_index) + 2;                            
                                else
                                    %落入小波门
                                    fusion.track_fusion_set(type_0_set(type_0_index)).track_quality(radar_index) = ...
                                        fusion.track_fusion_set(type_0_set(type_0_index)).track_quality(radar_index) + 3;                            
                                end
                            end
                            %融合
                            [fusion_X, fusion_P] = fusion.convexCombinationFusion(fusion.track_fusion_set(type_0_set(type_0_index)).X(:,end),...
                                fusion.track_fusion_set(type_0_set(type_0_index)).P,...
                                fusion.radar_track_set(loc).X, fusion.radar_track_set(loc).P);
                            fusion.track_fusion_set(type_0_set(type_0_index)).X(:,end) = fusion_X;
                            fusion.track_fusion_set(type_0_set(type_0_index)).P = fusion_P;                            
                        end
                    end
                end      
                
                %处理类型为1的航迹 使用GNN算法处理未连接的分站航迹
                if ~isempty(type_1_set)
                    %判断当前有无未关联的分站航迹
                    radar_track_index_set = [];
                    for radar_track_index = 1 : size(fusion.radar_track_set,2)
                        if fusion.radar_track_set(radar_track_index).connection_status == 0
                            radar_track_index_set = [radar_track_index_set ,...
                                radar_track_index];
                        end
                    end
                    if isempty(radar_track_index_set)
                        %若当前不存在可分配的分站航迹
                        for type_1_index = 1 : size(type_1_set,2)
                            fusion.track_fusion_set(type_1_set(type_1_index)).connection_status = 1;
                        end
                    else
                        %GNN 关联分配
                        [costMatrix,flagMatrix] = fusion.calCostMatrix(type_1_set, radar_track_index_set);%计算代价矩阵 代价矩阵中存储统计距离
                        [row ,loc] = linear_sum_assignment(costMatrix); 
                        
                        for i =1:length(row) 
                            if flagMatrix(i,loc(i)) == 1 %在大波门内 关联 融合
                                track_fusion_index = type_1_set(i);
                                radar_track_index = radar_track_index_set(loc(i));
                                fusion.track_fusion_set(track_fusion_index).connection_status=1;%关联成功
                                fusion.radar_track_set(radar_track_index).connection_status = 1;%关联成功
                                fusion.track_fusion_set(track_fusion_index).connection_table(radar_index) = radar_track_index;%将批号填入关联表
                                fusion.track_fusion_set(track_fusion_index).track_quality(radar_index) = 1;%初始化航迹质量
                                %融合
                                [fusion_X, fusion_P] = fusion.convexCombinationFusion(fusion.track_fusion_set(track_fusion_index).X(:,end),...
                                    fusion.track_fusion_set(track_fusion_index).P,...
                                    fusion.radar_track_set(radar_track_index).X, fusion.radar_track_set(radar_track_index).P);
                                fusion.track_fusion_set(track_fusion_index).X(:,end) = fusion_X;
                                fusion.track_fusion_set(track_fusion_index).P = fusion_P;          
                            else

                            end
                        end                        
                    end
                end                   
            case 2
                %当前是雷达2 的数据
                %优先处理类型为1的航迹
                if ~isempty(type_1_set)
                    for type_1_index = 1 : size(type_1_set,2)
                        %确认当前帧 雷达是否检测到该目标
                        [loc, value] = fusion.find_track(fusion.track_fusion_set(type_1_set(type_1_index)).connection_table(radar_index));
                        if value == 0
                            %若不存在 则认为目标消失 变为孤立航迹
                            fusion.track_fusion_set(type_1_set(type_1_index)).connection_status = 1;
                            fusion.track_fusion_set(type_1_set(type_1_index)).connection_table(radar_index) = -1;
                            fusion.track_fusion_set(type_1_set(type_1_index)).track_quality(radar_index) = -1;
                        else
                            %确认是否落入大小波门
                            fusion.track_fusion_set(type_1_set(type_1_index)).connection_status = 1;
                            fusion.radar_track_set(loc).connection_status = 1;

                            [~,flag] = fusion.Related_gate_track2track(type_1_set(type_1_index),loc,fusion.Chi_large);
                            if flag == 0
                                %没有落入大波门
                                fusion.track_fusion_set(type_1_set(type_1_index)).track_quality(radar_index) = ...
                                    fusion.track_fusion_set(type_1_set(type_1_index)).track_quality(radar_index) - 3;
                            else
                                %落入大波门 判断小波门
                                [~,flag1] = fusion.Related_gate_track2track(type_1_set(type_1_index),loc,fusion.Chi_small);
                                if flag1 == 0
                                    %没有落入小波门
                                    fusion.track_fusion_set(type_1_set(type_1_index)).track_quality(radar_index) = ...
                                        fusion.track_fusion_set(type_1_set(type_1_index)).track_quality(radar_index) + 2;                            
                                else
                                    %落入小波门
                                    fusion.track_fusion_set(type_1_set(type_1_index)).track_quality(radar_index) = ...
                                        fusion.track_fusion_set(type_1_set(type_1_index)).track_quality(radar_index) + 3;                            
                                end
                            end
                            %融合
                            [fusion_X, fusion_P] = fusion.convexCombinationFusion(fusion.track_fusion_set(type_1_set(type_1_index)).X(:,end),...
                                fusion.track_fusion_set(type_1_set(type_1_index)).P,...
                                fusion.radar_track_set(loc).X, fusion.radar_track_set(loc).P);
                            fusion.track_fusion_set(type_1_set(type_1_index)).X(:,end) = fusion_X;
                            fusion.track_fusion_set(type_1_set(type_1_index)).P = fusion_P;                            
                        end
                    end
                end      
                
                %处理类型为0的航迹 使用GNN算法处理未连接的分站航迹
                if ~isempty(type_0_set)
                    %判断当前有无未关联的分站航迹
                    radar_track_index_set = [];
                    for radar_track_index = 1 : size(fusion.radar_track_set,2)
                        if fusion.radar_track_set(radar_track_index).connection_status == 0
                            radar_track_index_set = [radar_track_index_set ,...
                                radar_track_index];
                        end
                    end
                    if isempty(radar_track_index_set)
                        %若当前不存在可分配的分站航迹
                        for type_0_index = 1 : size(type_0_set,2)
                            fusion.track_fusion_set(type_0_set(type_0_index)).connection_status = 1;
                        end
                    else
                        %GNN 关联分配
                        [costMatrix,flagMatrix] = fusion.calCostMatrix(type_0_set, radar_track_index_set);%计算代价矩阵 代价矩阵中存储统计距离
                        [row ,loc] = linear_sum_assignment(costMatrix); 
                        
                        for i =1:length(row) 
                            if flagMatrix(i,loc(i)) == 1 %在大波门内 关联 融合
                                track_fusion_index = type_0_set(i);
                                radar_track_index = radar_track_index_set(loc(i));
                                fusion.track_fusion_set(track_fusion_index).connection_status=1;%关联成功
                                fusion.radar_track_set(radar_track_index).connection_status = 1;%关联成功
                                fusion.track_fusion_set(track_fusion_index).connection_table(radar_index) = radar_track_index;%将批号填入关联表
                                fusion.track_fusion_set(track_fusion_index).track_quality(radar_index) = 1;%初始化航迹质量
                                %融合
                                [fusion_X, fusion_P] = fusion.convexCombinationFusion(fusion.track_fusion_set(track_fusion_index).X(:,end),...
                                    fusion.track_fusion_set(track_fusion_index).P,...
                                    fusion.radar_track_set(radar_track_index).X, fusion.radar_track_set(radar_track_index).P);
                                fusion.track_fusion_set(track_fusion_index).X(:,end) = fusion_X;
                                fusion.track_fusion_set(track_fusion_index).P = fusion_P;          
                            else

                            end
                        end                        
                    end
                end                       
        end
        
        %将未关联的分站航迹作为系统航迹起始
        radar_track_index_set = [];
        for radar_track_index = 1 : size(fusion.radar_track_set,2)
            if fusion.radar_track_set(radar_track_index).connection_status == 0
                radar_track_index_set = [radar_track_index_set ,...
                    radar_track_index];
            end
        end       
        if ~isempty(radar_track_index_set)
            num = fusion.track_fusion_num;
            for track_index = 1 : size(radar_track_index_set,2)
                radar_track = fusion.radar_track_set(radar_track_index_set(track_index));
                fusion.track_fusion_set = [fusion.track_fusion_set,...
                    Track_Fusion(radar_track.X, radar_track.P, ...
                    num + track_index,...
                    radar_track.track_report_index, radar_index)];
            end            
        end
    end   
end

