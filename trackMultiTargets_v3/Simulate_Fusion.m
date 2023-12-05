%% Simulate_Fusion
% Author: Shen Kailun
% Date  : 2023-11-16

%功能：融合跟踪
s = sprintf("====Simulate_Fusion====");
disp(s);

%依次处理各分站上报数据
for radar_index = 1 : radar_num
    %雷达上报
    fusion = radar_report(fusion,radar(radar_index),radar_index,origin_latitude,origin_longitude);
    %航迹处理
    %处理融合航迹
    if fusion.track_fusion_num == 0 %当前没有融合航迹
        fusion = track_init_fusion(fusion,frame_index);
    else %当前存在融合航迹
        %航迹处理
        fusion = track_process_GNN_fusion(fusion,frame_index,radar_index);
        
        %航迹管理
        fusion = track_manage_fusion(fusion, radar_index);
    end
    

end