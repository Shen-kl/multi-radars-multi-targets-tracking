%% Simulate_Initialise
% Author: Shen Kailun
% Date  : 2023-11-14

%功能：参数初始化 包含雷达系统参数初始化 跟踪器参数初始化等
s = sprintf("====Simulate_Initialise====");
disp(s);

%% 原点经纬高
origin_longitude = 120.61725;
origin_latitude = 31.336535;
%% 雷达参数初始化
radar_param = struct(...%雷达参数
    'location_enu',[],...%雷达坐标
    'location_geography',[],...%雷达经纬高
    'detection_distance_range',[],...%距离探测范围
    'detection_azimuth_range',[],...%方位角探测范围
    'detection_elevation_range',[],...%俯仰角探测范围
    'tracker_method',[],...'跟踪方法'
    'tracker',[],...'跟踪器'
    'T',[],...%帧周期
    'distance_measurement_error',[],...%距离观测误差
    'azimuth_measurement_error',[],...%方位观测误差
    'elevation_measurement_error',[],...%俯仰观测误差
    'measurement_set',[],...%量测集合
    'track_set',[],...%航迹集合
    'track_header',[]...%航迹头集合
);

%雷达1
radar_param(1).location_enu = [4e3 0 0];
radar_param(1).location_geography = [120.65519,31.336343,0];
radar_param(1).detection_distance_range = [1e3 5e3];
radar_param(1).detection_azimuth_range = [-12 12];
radar_param(1).detection_elevation_range = [0.5 80];
radar_param(1).tracker_method = 'KF';
radar_param(1).T = 0.1;
radar_param(1).distance_measurement_error = 20;
radar_param(1).azimuth_measurement_error = 0.2;
radar_param(1).elevation_measurement_error = 0.2;
%雷达2
radar_param(2).location_enu = [0 4e3 0];
radar_param(2).location_geography = [120.61725,31.371806,0];
radar_param(2).detection_distance_range = [1e3 6e3];
radar_param(2).detection_azimuth_range = [80 100];
radar_param(2).detection_elevation_range = [0.5 70];
radar_param(2).tracker_method = 'KF';
radar_param(2).T = 0.1;
radar_param(2).distance_measurement_error = 15;
radar_param(2).azimuth_measurement_error = 0.1;
radar_param(2).elevation_measurement_error = 0.15;

radar_num= size(radar_param, 2);%雷达数量

%探测区域
figure;
th = linspace(pi/2+radar_param(1).detection_azimuth_range(1) * pi / 180,...
pi/2+radar_param(1).detection_azimuth_range(2) * pi / 180,100);
x = radar_param(1).detection_distance_range(2)*cos(th)+radar_param(1).location_enu(1);
y = radar_param(1).detection_distance_range(2)*sin(th)+radar_param(1).location_enu(2);
plot([x,radar_param(1).location_enu(1),x(1)],[y,radar_param(1).location_enu(2),y(1)],'-k');
hold on;
th = linspace(pi/2+radar_param(1).detection_azimuth_range(1) * pi / 180,...
pi/2+radar_param(1).detection_azimuth_range(2) * pi / 180,100);
x = radar_param(1).detection_distance_range(1)*cos(th)+radar_param(1).location_enu(1);
y = radar_param(1).detection_distance_range(1)*sin(th)+radar_param(1).location_enu(2);
plot([x,radar_param(1).location_enu(1),x(1)],[y,radar_param(1).location_enu(2),y(1)],'-r');


th = linspace(-pi/2+radar_param(2).detection_azimuth_range(1) * pi / 180,...
-pi/2+radar_param(2).detection_azimuth_range(2) * pi / 180,100);
x = radar_param(2).detection_distance_range(2)*cos(th)+radar_param(2).location_enu(1);
y = radar_param(2).detection_distance_range(2)*sin(th)+radar_param(2).location_enu(2);
plot([x,radar_param(2).location_enu(1),x(1)],[y,radar_param(2).location_enu(2),y(1)],'-k');
th = linspace(-pi/2+radar_param(2).detection_azimuth_range(1) * pi / 180,...
-pi/2+radar_param(2).detection_azimuth_range(2) * pi / 180,100);
x = radar_param(2).detection_distance_range(1)*cos(th)+radar_param(2).location_enu(1);
y = radar_param(2).detection_distance_range(1)*sin(th)+radar_param(2).location_enu(2);
plot([x,radar_param(2).location_enu(1),x(1)],[y,radar_param(2).location_enu(2),y(1)],'-r');
grid on;
title('双基地雷达探测范围');
xlabel('m');
ylabel('m');

%% 滤波器初始化
for index = 1 : radar_num
    switch radar_param(index).tracker_method % 可以扩充
        case 'KF'
            T = radar_param(index).T;
            F=[1 T 1/2*T^2 0 0 0 0 0 0 ;...
                0 1 T 0 0 0 0 0 0;...
                0 0 1 0 0 0 0 0 0;...
                0 0 0 1 T 1/2*T^2 0 0 0;...
                0 0 0 0 1 T 0 0 0;...
                0 0 0 0 0 1 0 0 0;...
                0 0 0 0 0 0 1 T T^2/2;...
                0 0 0 0 0 0 0 1 T;...
                0 0 0 0 0 0 0 0 1];%状态转移矩阵
            W=[50,0.3*pi/180,0.3*pi/180]';%量测噪声矩阵 x y z轴
            R=[100^2 0 0;0 (0.3*pi/180)^2 0;0 0 (0.3*pi/180)^2];%W的协方差矩阵 

            nvar=10;%系统噪声
            Q=nvar*[(T^5/20) (T^4/8)  (T^3/6) zeros(1,6);...
                        (T^4/8) (T^3/2) (T^2/2) zeros(1,6);
                        (T^3/6) (T^2/2) T zeros(1,6);
                    zeros(1,3) (T^5/20) (T^4/8)  (T^3/6) zeros(1,3);
                    zeros(1,3) (T^4/8) (T^3/2)  (T^2/2) zeros(1,3);
                    zeros(1,3) (T^3/6) (T^2/2) T zeros(1,3);...
                    zeros(1,6) (T^5/20) (T^4/8)  (T^3/6);
                    zeros(1,6) (T^4/8) (T^3/2) (T^2/2);
                    zeros(1,6) (T^3/6) (T^2/2) T];%过程噪声  
           H=[1 0 0 0 0 0 0 0 0 0;
               0 0 0 1 0 0 0 0 0 0;
               0 0 0 0 0 0 1 0 0 0];%观测矩阵
           radar_param(index).tracker =  KalmanFilter(F, H, Q, R);
        case 'EKF'
            
        case 'UKF'    
            
    end

end

%% 航迹号分配
TarTrackIndex=zeros(1,1000);%目标航迹号
for i=1:1000
    TarTrackIndex(i)=i;
end

HeadIndex=zeros(1,1000);%航迹头标号
for i=1:1000
    HeadIndex(i)=i;
end

%% 航迹起始参数设置
lambda_NT=1e-6;%新目标空间密度
lambda_FA=64e-4;%杂波空间密度
Pd=0.99;%检测概率
Pf=1e-6;%虚警概率
trackProbabilityThreshold = 0.6;
Vmin=[-300;-300;-300];%目标最小速度 X方向 Y方向
Vmax=[300;300;300];%目标最大速度 X方向 Y方向
PG=1;%门概率质量
Chi_large=16;%对应2维 门概率质量PG=1
Chi_small=5;%对应2维 门概率质量PG 

RULE = "2_2" ; %起始准则