%% Simulate_Plot
% Author: Shen Kailun
% Date  : 2023-11-16

%功能：绘图
s = sprintf("====Simulate_Plot====");
disp(s);

%探测区域
figure(1);
th = linspace(pi/2+(radar(1).detection_azimuth_range(1)) * pi / 180,...
pi/2+(radar(1).detection_azimuth_range(2)) * pi / 180,100);
x = radar(1).detection_distance_range(2)*cos(th)+radar(1).location_enu(1);
y = radar(1).detection_distance_range(2)*sin(th)+radar(1).location_enu(2);
plot([x,radar(1).location_enu(1),x(1)],[y,radar(1).location_enu(2),y(1)],'-k');
hold on;
th = linspace(pi/2+(radar(1).detection_azimuth_range(1)) * pi / 180,...
pi/2+(radar(1).detection_azimuth_range(2)) * pi / 180,100);
x = radar(1).detection_distance_range(1)*cos(th)+radar(1).location_enu(1);
y = radar(1).detection_distance_range(1)*sin(th)+radar(1).location_enu(2);
plot([x,radar(1).location_enu(1),x(1)],[y,radar(1).location_enu(2),y(1)],'-r');


th = linspace(-pi/2+radar(2).detection_azimuth_range(1) * pi / 180,...
-pi/2+radar(2).detection_azimuth_range(2) * pi / 180,100);
x = radar(2).detection_distance_range(2)*cos(th)+radar(2).location_enu(1);
y = radar(2).detection_distance_range(2)*sin(th)+radar(2).location_enu(2);
plot([x,radar(2).location_enu(1),x(1)],[y,radar(2).location_enu(2),y(1)],'-k');
th = linspace(-pi/2+radar(2).detection_azimuth_range(1) * pi / 180,...
-pi/2+radar(2).detection_azimuth_range(2) * pi / 180,100);
x = radar(2).detection_distance_range(1)*cos(th)+radar(2).location_enu(1);
y = radar(2).detection_distance_range(1)*sin(th)+radar(2).location_enu(2);
plot([x,radar(2).location_enu(1),x(1)],[y,radar(2).location_enu(2),y(1)],'-r');
grid on;
title('双基地雷达探测范围');
xlabel('m');
ylabel('m');

%雷达 目标
marker = ["go","ko"];
marker_plot = ["g*","k*"];
legend_set_ori = {'radar1-track','radar1-plot-track','radar2-track','radar2-plot-track','fusion-track'};
handle_set = [];
legend_set = [];
for radar_index = 1 : radar_num
    for track_index = 1 : radar(radar_index).track_num
        [X,Y,Z] = enu2ecef(radar(radar_index).track_set(track_index).X(1,:),...
            radar(radar_index).track_set(track_index).X(4,:),...
            radar(radar_index).track_set(track_index).X(7,:),...
            radar(radar_index).location_geography(2),...
            radar(radar_index).location_geography(1),...
            radar(radar_index).location_geography(3),wgs84Ellipsoid);
        [xEast,yNorth,zUp] = ecef2enu(X,Y,Z,...
            origin_latitude,...
            origin_longitude,...
            origin_height,wgs84Ellipsoid);
        track_handle(radar_index) = plot(xEast,...
            yNorth,marker(radar_index));
        text(xEast(end),yNorth(end),num2str(radar(radar_index).track_set(track_index).track_index)); 
    end
    if radar(radar_index).track_num > 0
        handle_set = [handle_set track_handle(radar_index)];
        legend_set = [legend_set legend_set_ori(2* (radar_index - 1)+ 1)];
    end
    for track_index = 1 : radar(radar_index).plot_track_num
        [X,Y,Z] = enu2ecef(radar(radar_index).plot_track_set(track_index).X(1,:),...
            radar(radar_index).plot_track_set(track_index).X(2,:),...
            radar(radar_index).plot_track_set(track_index).X(3,:),...
            radar(radar_index).location_geography(2),...
            radar(radar_index).location_geography(1),...
            radar(radar_index).location_geography(3),wgs84Ellipsoid);
        [xEast,yNorth,zUp] = ecef2enu(X,Y,Z,...
            origin_latitude,...
            origin_longitude,...
            origin_height,wgs84Ellipsoid);        
        plot_track_handle(radar_index) = plot(xEast,...
            yNorth,marker_plot(radar_index));
    end       
    if radar(radar_index).plot_track_num > 0
        handle_set = [handle_set plot_track_handle(radar_index)];
        legend_set = [legend_set legend_set_ori(2 * radar_index)];
    end    
end

for track_fusion_index = 1 : fusion.track_fusion_num   
    track_fusion_handle = plot(fusion.track_fusion_set(track_fusion_index).X(1,:),...
        fusion.track_fusion_set(track_fusion_index).X(4,:),"r>");
    text(fusion.track_fusion_set(track_fusion_index).X(1,end),...
        fusion.track_fusion_set(track_fusion_index).X(4,end),...
    num2str(fusion.track_fusion_set(track_fusion_index).track_fusion_index));    
end  
if fusion.track_fusion_num > 0
    handle_set = [handle_set track_fusion_handle];
    legend_set = [legend_set legend_set_ori(5)];
end    
if ~isempty(handle_set)
    legend(handle_set,...
        legend_set);
end
hold off;
% figure(1);
% hold on;
% for track_index = 1 : radar(1).track_num
%     plot(radar(1).track_set(track_index).X(1,end),...
%         radar(1).track_set(track_index).X(4,end));
% end
% for track_index = 1 : radar(1).plot_track_num
%     plot(radar(1).plot_track_set(track_index).X(1,end),...
%         radar(1).plot_track_set(track_index).X(2,end),'*');
% end
% set (gcf,'position', [800 300 500 500]);
% figure(2);
% hold on;
% for track_index = 1 : radar(2).track_num
%     plot(radar(2).track_set(track_index).X(1,end),...
%         radar(2).track_set(track_index).X(4,end));
% end
% for track_index = 1 : radar(2).plot_track_num
%     plot(radar(2).plot_track_set(track_index).X(1,:),...
%         radar(2).plot_track_set(track_index).X(2,:),'*');
% end
% set (gcf,'position', [0 300 500 500]);