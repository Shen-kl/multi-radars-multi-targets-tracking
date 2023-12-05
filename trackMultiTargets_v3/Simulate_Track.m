%% Simulate_Track
% Author: Shen Kailun
% Date  : 2023-11-14

%功能： 雷达跟踪
s = sprintf("====Simulate_Track====");
disp(s);

for radar_index = 1 : radar_num    
    %分区
    radar(radar_index) = zone(radar(radar_index));
    %航迹处理
    radar(radar_index) = track_process_GNN(radar(radar_index));
    %航迹起始
    radar(radar_index) = track_init(radar(radar_index));
    %航迹管理
    radar(radar_index) = track_manage(radar(radar_index));
end