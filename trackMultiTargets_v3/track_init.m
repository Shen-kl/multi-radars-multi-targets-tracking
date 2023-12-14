% Author: Shen Kailun
% Date  : 2023-11-14
function [radar] = track_init(radar)
%函数功能： 航迹起始
    tracksHead_LASTMOMENT = radar.track_header_set;
    tracksHeadNum_LASTMOMENT = radar.track_header_num;
    tracker_method = radar.tracker_method;
    T = radar.T;
    lambda_NT = radar.lambda_NT;
    lambda_FA = radar.lambda_FA;
    track_head_num = 0;
    Vmax = radar.Vmax;
    Vmin = radar.Vmin;
    radar.track_header_set = [];
    R = radar.tracker.R;
    Pd = radar.Pd;
    Pf = radar.Pf;
    distance_step = radar.distance_step;
    azi_step = radar.azi_step;
    distance_max = radar.detection_distance_range(2);
    distance_min = radar.detection_distance_range(1);
    azi_max = radar.detection_azimuth_range(2);
    azi_min = radar.detection_azimuth_range(1);    
    Chi_large = radar.Chi_large;
    PG = radar.PG;
    if tracksHeadNum_LASTMOMENT == 0 %若航迹头集合为空 将全部量测点列为航迹头
        for j=1 : radar.plot_track_num
            if radar.plot_track_set(j).connection_status == 0 %未被使用
                track_head_num = track_head_num + 1;
                radar.track_header_set = [radar.track_header_set Track_Header(track_head_num,...
                                                        radar.plot_track_set(j).X,...
                                                        radar.plot_track_set(j).Polar_X,...
                                                        radar.plot_track_set(j).zone_index,...
                                                        lambda_NT/lambda_FA)];
            end
        end
    else    
        for j=1:tracksHeadNum_LASTMOMENT
            tracksHead_LASTMOMENT(j).connection_status=0;
            [locSet] = Sudoku(tracksHead_LASTMOMENT(j).zone_index, distance_step, azi_step, distance_max, distance_min, azi_max, azi_min);%查找要搜寻的所有分区编号
            for k =1 : length(locSet)%逐分区搜索
                if radar.At_Location_Plot_Track(locSet(k)).plot_track_num > 0 %若当前分区有点迹存在
                    for m = 1 : radar.At_Location_Plot_Track(locSet(k)).plot_track_num
                        index_temp=radar.At_Location_Plot_Track(locSet(k)).plot_track_index(m);%提取航迹号
                        if radar.plot_track_set(index_temp).connection_status == 0 %未被使用
                            d=max(zeros(3,1),radar.plot_track_set(index_temp).X-tracksHead_LASTMOMENT(j).X(:,end)-T*Vmax)+max(zeros(3,1),-radar.plot_track_set(index_temp).X+tracksHead_LASTMOMENT(j).X(:,end)+T*Vmin);
                            switch tracker_method
                                case 'KF'
                                    R_sum = R+R;
                                case 'EKF'
                                    R_1=error_conversion(R,radar.plot_track_set(index_temp).Polar_X);
                                    R_2=error_conversion(R,tracksHead_LASTMOMENT(j).Polar_X);
                                    R_sum = R_1+R_2;
                                case 'Singer'
                                    R_1=error_conversion(R,radar.plot_track_set(index_temp).Polar_X);
                                    R_2=error_conversion(R,tracksHead_LASTMOMENT(j).Polar_X);
                                    R_sum = R_1+R_2;
                                case 'UKF'
                                    R_1=error_conversion(R,radar.plot_track_set(index_temp).Polar_X);
                                    R_2=error_conversion(R,tracksHead_LASTMOMENT(j).Polar_X);
                                    R_sum = R_1+R_2;                                    
                            end
                            D = d'*R_sum*d;

                            if D<=Chi_large %量测互联 若落入波门 建立可能航迹 否则不建立
                                track_head_num = track_head_num + 1;   

                                VG=pi*PG*sqrt(det(R_sum));
                                L=(Pd*VG*exp(-D/2))/(Pf*(2*pi)^(2/2)*sqrt(det(R_sum)));

%                                 VG=pi*PG*sqrt(det(R+R));
%                                 L=(Pd*VG*exp(-D/2))/(Pf*(2*pi)^(2/2)*sqrt(det(R+R)));
                                likelihoodProbability = (L*tracksHead_LASTMOMENT(j).likelihoodProbability)/(L*tracksHead_LASTMOMENT(j).likelihoodProbability +1 - tracksHead_LASTMOMENT(j).likelihoodProbability);%更新似然比
                                tracksHead_LASTMOMENT(j).connection_status=1;
                                radar.track_header_set = [radar.track_header_set Track_Header(track_head_num,...
                                                                        [ tracksHead_LASTMOMENT(j).X radar.plot_track_set(index_temp).X],...
                                                                        [ tracksHead_LASTMOMENT(j).Polar_X radar.plot_track_set(index_temp).Polar_X],...
                                                                        radar.plot_track_set(index_temp).zone_index,...
                                                                        likelihoodProbability)];    
                                radar.plot_track_set(index_temp).inDoor=1;%落入相关波门   
                                radar.track_header_set(track_head_num).track_quality=tracksHead_LASTMOMENT(j).track_quality+1;%航迹质量更新
                            end
                        end                        
                     end
                end
            end
            if tracksHead_LASTMOMENT(j).connection_status==0%如果临时航迹在该时刻没有关联到点迹 则做外推
                track_head_num = track_head_num + 1;
                L=(1-Pd)/(1-Pf);
                likelihoodProbability = (L*tracksHead_LASTMOMENT(j).likelihoodProbability)/(L*tracksHead_LASTMOMENT(j).likelihoodProbability +1 - tracksHead_LASTMOMENT(j).likelihoodProbability);%更新似然比
                tracksHead_LASTMOMENT(j).connection_status=1;
                radar.track_header_set = [radar.track_header_set Track_Header(track_head_num,...
                                                        [ tracksHead_LASTMOMENT(j).X tracksHead_LASTMOMENT(j).X],...
                                                        [ tracksHead_LASTMOMENT(j).Polar_X tracksHead_LASTMOMENT(j).Polar_X],...
                                                        tracksHead_LASTMOMENT(j).zone_index,...
                                                        likelihoodProbability)];      
                radar.track_header_set(track_head_num).track_quality=tracksHead_LASTMOMENT(j).track_quality - 3;%航迹质量更新
            end          
        end
        for k=1 : radar.plot_track_num
            if radar.plot_track_set(k).connection_status==0 && radar.plot_track_set(k).inDoor==0%未被使用 且 未落入任何一个相关波门 设为新航迹头
                track_head_num = track_head_num +1;
                radar.track_header_set = [radar.track_header_set Track_Header(track_head_num,...
                                                        radar.plot_track_set(k).X,...
                                                        radar.plot_track_set(k).Polar_X,...
                                                        radar.plot_track_set(k).zone_index,...
                                                        lambda_NT/lambda_FA)];                
            end
        end

    end    
    radar.track_header_num = track_head_num;
end

