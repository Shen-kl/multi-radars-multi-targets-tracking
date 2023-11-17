% Author: Shen Kailun
% Date  : 2023-11-15
function [radar] = track_process(radar)
%函数功能： 航迹处理 关联滤波 最近邻处理
    distance_step = radar.distance_step;
    azi_step = radar.azi_step;
    distance_max = radar.detection_distance_range(2);
    distance_min = radar.detection_distance_range(1);
    azi_max = radar.detection_azimuth_range(2);
    azi_min = radar.detection_azimuth_range(1);   
    
    for track_index = 1 : radar.track_num
        if radar.track_set(track_index).track_property ~= 2
            [locSet] = Sudoku(radar.track_set(track_index).zone_index, distance_step, azi_step, distance_max, distance_min, azi_max, azi_min);%查找要搜寻的所有分区编号
            DSet = [];
            IndexSet= [];
            for j =1 : length(locSet)%逐分区搜索
                if radar.At_Location_Plot_Track(locSet(j)).plot_track_num > 0 %若当前分区有点迹存在
                    for k = 1 : radar.At_Location_Plot_Track(locSet(j)).plot_track_num
                        index_temp=radar.At_Location_Plot_Track(locSet(j)).plot_track_index(k);%提取航迹号
                        if radar.plot_track_set(index_temp).connection_status == 0 %未关联
                            [D,flag] = radar.Related_gate_track2ob(track_index,index_temp,radar.Chi_large);%相关波门判断 大波门
                            if flag==1
                                DSet=[DSet D];
                                IndexSet=[IndexSet index_temp];
                            end   
                        end
                    end
                end
            end
            if isempty(DSet) %没有点迹落入波门
                radar.track_set(track_index).connection_status=0;
                radar.track_set(track_index).track_quality = radar.track_set(track_index).track_quality - 3 ;
            else  %有点迹落入波门  取最近的点迹
                [~ ,loc]=min(DSet);
                radar.track_set(track_index).connection_status=1;
                radar.plot_track_set(IndexSet(loc)).connection_status = 1;
                [~,flag1] = radar.Related_gate_track2ob(track_index,IndexSet(loc),radar.Chi_small);%相关波门 小波门
                if flag1 == 0 %未进入小波门
                    radar.track_set(track_index).track_quality = radar.track_set(track_index).track_quality + 2 ;
                else %进入小波门
                    radar.track_set(track_index).track_quality = radar.track_set(track_index).track_quality + 3 ;
                end
                [radar.track_set(track_index).X(:,end), radar.track_set(track_index).P] = radar.tracker.KalmanUpdate(radar.track_set(track_index).X(:,end), radar.track_set(track_index).P, radar.plot_track_set(IndexSet(loc)).X);%卡尔曼更新
            end 
        end
    end

end

