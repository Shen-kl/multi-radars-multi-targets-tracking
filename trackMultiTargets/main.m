%% 多目标跟踪
clear all;close all;clc;
%% 初始化
Simulate_Initialise;

%% 加载目标仿真轨迹
Simulate_Targets;

%% 数据处理

for frame_index = 1 : frame_max
    %产生目标量测
    Simulate_Measurements;
    
    %单雷达目标跟踪
    
    
    %融合
    Simulate_Fusion;
    
    %绘图

end

