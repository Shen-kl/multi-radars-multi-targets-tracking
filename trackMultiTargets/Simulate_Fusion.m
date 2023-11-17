%% Simulate_Fusion
% Author: Shen Kailun
% Date  : 2023-11-16

%功能：融合跟踪
s = sprintf("====Simulate_Fusion====");
disp(s);

%分区
fusion = zone(fusion);
%航迹处理
fusion = track_process_GNN(fusion);
%航迹起始
fusion = track_init(fusion);
%航迹管理
fusion = track_manage(fusion);