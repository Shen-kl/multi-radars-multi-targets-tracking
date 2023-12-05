% Author: Shen Kailun
% Date  : 2023-11-19
function [fusion] = track_manage_fusion(fusion, radar_index)
    %函数功能： 航迹管理 航迹类型切换
    DeleteTrackSet = [];
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

    %处理类型2航迹
    for type_2_index = 1 : size(type_2_set,2)
        if fusion.track_fusion_set(type_2_set(type_2_index)).track_quality(radar_index) > 12
            fusion.track_fusion_set(type_2_set(type_2_index)).track_quality(radar_index) = 12;
        elseif fusion.track_fusion_set(type_2_set(type_2_index)).track_quality(radar_index)< 0
            %切换为孤立航迹
            fusion.track_fusion_set(type_2_set(type_2_index)).connection_table(radar_index) = -1;
            fusion.track_fusion_set(type_2_set(type_2_index)).track_quality(radar_index) = -1;
            tmp = [1,0];
            fusion.track_fusion_set(type_2_set(type_2_index)).track_type = tmp(radar_index);
        end
    end
    
    switch radar_index
        case 1
            %处理类型0航迹
            for type_0_index = 1 : size(type_0_set,2)
                if fusion.track_fusion_set(type_0_set(type_0_index)).track_quality(radar_index) > 12
                    fusion.track_fusion_set(type_0_set(type_0_index)).track_quality(radar_index) = 12;
                elseif fusion.track_fusion_set(type_0_set(type_0_index)).track_quality(radar_index)< 0
                    %删除航迹
                    DeleteTrackSet = [DeleteTrackSet type_0_set(type_0_index)];
                end
            end     
            %处理类型1航迹
            for type_1_index = 1 : size(type_1_set,2)
                if fusion.track_fusion_set(type_1_set(type_1_index)).track_quality(1) > 0
                    %类型切换
                    fusion.track_fusion_set(type_1_set(type_1_index)).track_type = 2;
                end
                if fusion.track_fusion_set(type_1_set(type_1_index)).aloneCnt == 6 %连续六帧没关联上
                    %删除航迹
                    DeleteTrackSet = [DeleteTrackSet type_1_set(type_1_index)];             
                end
            end                  
        case 2
            %处理类型0航迹
            for type_0_index = 1 : size(type_0_set,2)
                if fusion.track_fusion_set(type_0_set(type_0_index)).track_quality(2) > 0
                    %类型切换
                    fusion.track_fusion_set(type_0_set(type_0_index)).track_type = 2;
                end
                if fusion.track_fusion_set(type_0_set(type_0_index)).aloneCnt == 6 %连续六帧没关联上
                    %删除航迹
                    DeleteTrackSet = [DeleteTrackSet type_0_set(type_0_index)];             
                end                
            end     
            %处理类型1航迹
            for type_1_index = 1 : size(type_1_set,2)
                if fusion.track_fusion_set(type_1_set(type_1_index)).track_quality(radar_index) > 12
                    fusion.track_fusion_set(type_1_set(type_1_index)).track_quality(radar_index) = 12;
                elseif fusion.track_fusion_set(type_1_set(type_1_index)).track_quality(radar_index)< 0
                    %删除航迹
                    DeleteTrackSet = [DeleteTrackSet type_1_set(type_1_index)];
                end
            end              
    end

    %删除质量低的航迹
    if ~isempty(DeleteTrackSet)
        fusion.track_fusion_set(DeleteTrackSet) =[];
    end
    
    fusion.track_fusion_num = size(fusion.track_fusion_set,2);%更新融合航迹数量
end

