% Author: Shen Kailun
% Date  : 2023-11-14
classdef Radar
    %雷达    
    properties
        location_enu %雷达坐标
        location_geography %雷达经纬高
        detection_distance_range %距离探测范围
        detection_azimuth_range %方位角探测范围
        detection_elevation_range %俯仰角探测范围
        tracker_method %'跟踪方法'
        tracker %跟踪器
        T %帧周期
        distance_measurement_error %距离观测误差
        azimuth_measurement_error %方位观测误差
        elevation_measurement_error %俯仰观测误差
        plot_track_set %点迹集合
        track_set %航迹集合
        track_header_set  %航迹头集合
        track_num %航迹数量
        track_header_num%航迹头数量
        plot_track_num%点迹数量
        lambda_NT%新目标空间密度
        lambda_FA%杂波空间密度
        Pd%检测概率
        Pf%虚警概率
        trackProbabilityThreshold
        Vmin%目标最小速度 X方向 Y方向
        Vmax%目标最大速度 X方向 Y方向
        PG%门概率质量
        Chi_large%对应2维 门概率质量PG=1
        Chi_small%对应2维 门概率质量PG 
        distance_step%分区距离步长
        azi_step%分区方位角步长
        At_Location_Plot_Track
        track_index_set %可用批号
        head_index_set %可用点迹标号
        report %上报航迹
    end
    
    methods
        function obj = Radar(location_enu, location_geography,...
                detection_distance_range, detection_azimuth_range,...
                detection_elevation_range, tracker_method,...
                T,distance_measurement_error,azimuth_measurement_error,...
                elevation_measurement_error,lambda_NT,lambda_FA,Pd,...
                Pf,trackProbabilityThreshold,Vmin,Vmax,PG,Chi_large,...
                Chi_small,distance_step,azi_step)
            obj.location_enu = location_enu;
            obj.location_geography = location_geography;
            obj.detection_distance_range = detection_distance_range;
            obj.detection_azimuth_range = detection_azimuth_range;
            obj.detection_elevation_range = detection_elevation_range;
            obj.tracker_method = tracker_method;
            obj.T = T;
            obj.distance_measurement_error = distance_measurement_error;
            obj.azimuth_measurement_error = azimuth_measurement_error;
            obj.elevation_measurement_error = elevation_measurement_error;
            obj.lambda_NT = lambda_NT;
            obj.lambda_FA = lambda_FA;
            obj.Pd = Pd;
            obj.Pf = Pf;
            obj.trackProbabilityThreshold = trackProbabilityThreshold;
            obj.Vmin = Vmin;
            obj.Vmax = Vmax;
            obj.PG = PG;
            obj.Chi_large = Chi_large;
            obj.Chi_small = Chi_small;
            obj.distance_step = distance_step;
            obj.azi_step = azi_step;
            obj.track_header_num = 0;
            obj.track_num = 0;  
            
            obj.track_index_set=zeros(1,1000);%目标航迹号
            for i=1:1000
                obj.track_index_set(i)=i;
            end

            obj.head_index_set=zeros(1,1000);%航迹头标号
            for i=1:1000
                obj.head_index_set(i)=i;
            end            
        end
        
        function [x,value]=find_head(obj,num)
        %根据航迹号找对应的航迹头
        %input num 想要找的本地航迹的航迹编号
        %output x 该本地航迹在track_header_set中数组的序号
        %output value 1:成功找到 0：未找到
            value=0;
            for x=1:obj.track_header_num
                if obj.track_header_set(x).header_index==num
                    value=1;
                    break;
                end
            end
        end       

        function [x,value]=find_track(obj,num)
        %根据航迹号找对应的航迹
        %input num 想要找的本地航迹的航迹编号
        %output x 该本地航迹在track_header_set中数组的序号
        %output value 1:成功找到 0：未找到
            value=0;
            for x=1:obj.track_num
                if obj.track_set(x).track_index==num
                    value=1;
                    break;
                end
            end
        end     
        function [D,flag] = Related_gate_track2ob(obj,track_index,plot_track_index,Chi)
            %相关波门判断
            %input track_index 此条本地航迹在数组中的序列号
            %input plot_track_index 此条系统航迹在数组中的序列号    
            %input Chi 门限常数

            %output D 统计距离
            %output flag 1 :相关 0：不相关
%             % 卡尔曼滤波器对应的椭圆跟踪门面积
%             S=obj.tracker.H*(obj.track_set(track_index).P)*obj.tracker.H'+ obj.tracker.R; %计算椭圆跟踪门的面积
            % 扩展卡尔曼滤波器对应的椭圆跟踪门面积
%             jacobian_H = double(subs(obj.tracker.jacobian_H,{'x','x_1','x_2','y','y_1','y_2','z','z_1','z_2'}, ...
%                 {obj.track_set(track_index).X(1),obj.track_set(track_index).X(2),obj.track_set(track_index).X(3), ...
%                 obj.track_set(track_index).X(4),obj.track_set(track_index).X(5),obj.track_set(track_index).X(6), ...
%                 obj.track_set(track_index).X(7),obj.track_set(track_index).X(8),obj.track_set(track_index).X(9)}));
%             jacobian_H = obj.tracker.jacobian_H(obj.track_set(track_index).X(1), ...
%                 obj.track_set(track_index).X(4), ...
%                 obj.track_set(track_index).X(7));
            jacobian_H = mearsure_jacobian(obj.track_set(track_index).X(1), ...
                obj.track_set(track_index).X(4), ...
                obj.track_set(track_index).X(7));
            S=jacobian_H*(obj.track_set(track_index).P)*jacobian_H'+ obj.tracker.R;

        %     ellipse_Volume=pi*Chi*sqrt(det(S));                             
            d=obj.track_set(track_index).X(1:3:9,end)-obj.plot_track_set(plot_track_index).X;%预测状态向量与观测值的残差  

            D=d'*inv(S)*d;
            if D<0
                disp('error')
            end
            if D<=Chi 
                flag=1;
            else
                flag=0;
            end
        end  
        
        function [costMatrix,flagMatrix] = calCostMatrix(obj,clusterSet, obIndexSet)
        %计算代价矩阵 即欧氏距离矩阵
        %矩阵沿着列方向为航迹 行方向为观测点迹
        costMatrix = zeros(length(clusterSet),length(obIndexSet));
        flagMatrix = zeros(length(clusterSet),length(obIndexSet));
        for i =1 : length(clusterSet)
            for j = 1 : length(obIndexSet)
                [D,flag] = obj.Related_gate_track2ob(clusterSet(i),obIndexSet(j),obj.Chi_large);
                costMatrix(i,j) = D;
                flagMatrix(i,j) = flag;
            end
        end
        end        
    end
end

