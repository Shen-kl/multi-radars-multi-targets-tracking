% Author: Shen Kailun
% Date  : 2023-11-15
function [radar] = track_process_GNN(radar)
%函数功能： 航迹处理 关联滤波 GNN
    distance_step = radar.distance_step;
    azi_step = radar.azi_step;
    tracker_method = radar.tracker_method;
    distance_max = radar.detection_distance_range(2);
    distance_min = radar.detection_distance_range(1);
    azi_max = radar.detection_azimuth_range(2);
    azi_min = radar.detection_azimuth_range(1);   
     
    locSet = [];
    clusterSet=[];
    for i =1 : radar.track_num
        if radar.track_set(i).track_property ~=2%判断是否是终结航迹
            clusterSet= [clusterSet i];
            zone_index = radar.track_set(i).zone_index;
            [locSetTemp] = Sudoku(zone_index, distance_step, azi_step, distance_max,distance_min,...
                azi_max,azi_min);
            locSet = union(locSet,locSetTemp);
        end
    end
    
    if ~isempty(clusterSet) %若当前存在非终结航迹
%         plot_track_index_set = [];
%         for i = 1 :length(locSet) %查找同分区以及相邻分区的点迹
%             for j =1:radar.At_Location_Plot_Track(locSet(i)).plot_track_num
%                 index = radar.At_Location_Plot_Track(locSet(i)).plot_track_index(j);
%                 if radar.plot_track_set(index).connection_status == 0%未被使用过
%                     plot_track_index_set = [plot_track_index_set index];
%                 end
%             end
%         end
        if radar.plot_track_num ~= 0 %若当前存在点迹
            plot_track_index_set = linspace(1,radar.plot_track_num,radar.plot_track_num);
            [costMatrix,flagMatrix] = radar.calCostMatrix(clusterSet, plot_track_index_set);%计算代价矩阵 代价矩阵中存储统计距离
            [row ,loc] = linear_sum_assignment(costMatrix);

            for i =1:length(row) 
                if flagMatrix(row(i),loc(i)) == 1
                    track_index = clusterSet(row(i));
                    plot_track_index = plot_track_index_set(loc(i));
                    radar.track_set(track_index).connection_status=1;
                    radar.plot_track_set(plot_track_index).connection_status = 1;
                    [~,flag1] = radar.Related_gate_track2ob(track_index,plot_track_index,radar.Chi_small);%相关波门 小波门
                    if flag1 == 0 %未进入小波门
                        radar.track_set(track_index).track_quality = radar.track_set(track_index).track_quality + 2 ;
                    else %进入小波门
                        radar.track_set(track_index).track_quality = radar.track_set(track_index).track_quality + 3 ;
                    end
                    switch tracker_method
                        case 'KF'
                            [radar.track_set(track_index).X(:,end), radar.track_set(track_index).P] = radar.tracker.KalmanUpdate...
                                (radar.track_set(track_index).X(:,end), radar.track_set(track_index).P, radar.plot_track_set(plot_track_index).X);%卡尔曼更新
                        case 'EKF'
                            [radar.track_set(track_index).X(:,end), radar.track_set(track_index).P] = radar.tracker.ExtentedKalmanUpdate...
                                (radar.track_set(track_index).X(:,end), radar.track_set(track_index).P, radar.plot_track_set(plot_track_index).Polar_X);%扩展卡尔曼更新
                        case 'Singer'
                            [radar.track_set(track_index).X(:,end), radar.track_set(track_index).P] = radar.tracker.SingerUpdate...
                                (radar.track_set(track_index).X(:,end), radar.track_set(track_index).P, radar.plot_track_set(plot_track_index).X);%Singer更新
                    end
                else
                    track_index = clusterSet(row(i));
                    radar.track_set(track_index).connection_status=0;
                    radar.track_set(track_index).track_quality = radar.track_set(track_index).track_quality - 3 ;
                end
            end
            
            %处理未被分配的航迹
            uncorr = setdiff(clusterSet', row);%求出clusterSet内 不在row中的元素
            for index = 1 : length(uncorr)
                track_index = uncorr(index);
                radar.track_set(track_index).connection_status=0;
                radar.track_set(track_index).track_quality = radar.track_set(track_index).track_quality - 3 ;                
            end
        else
            for track_index =1 : radar.track_num
                if radar.track_set(track_index).track_property ~=2%判断是否是终结航迹
                    radar.track_set(track_index).connection_status=0;
                    radar.track_set(track_index).track_quality = radar.track_set(track_index).track_quality - 3 ;
                end
            end            
        end
    end
end

