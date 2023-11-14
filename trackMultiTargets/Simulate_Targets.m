%% Simulate_Targets
% Author: Shen Kailun
% Date  : 2023-11-14

%功能：仿真目标轨迹
s = sprintf("====Simulate_Targets====");
disp(s);

%加载目标轨迹
load("target.mat");
target_num = 4;%要加载的目标数量

for index = 1 : target_num
    exp = ['target_set{' num2str(index) '}=target' num2str(index) ';'];
    eval(exp);
end

%计算最大跟踪帧数
frame_max = 0;
for index = 1 : target_num
    exp = ['frame_max = max(frame_max, size(target' num2str(index) ', 1))' ';'];
    eval(exp);
end