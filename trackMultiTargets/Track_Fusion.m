% Author: Shen Kailun
% Date  : 2023-11-17
classdef Track_Fusion
    %融合航迹
    properties
        X %状态向量
        P %误差协方差矩阵
        Polar_X %极坐标
        track_quality %航迹得分
        connection_status %关联状态 0：未关联 1：关联成功
        track_fusion_index %批号
        zone_index
        track_property %0：滤波 1：外推 2：终结
        connection_table %关联表
        track_type % 0:与雷达1匹配上 1：与雷达2匹配上 3：与雷达1、2匹配上
    end
    
    methods
        function obj = Track_Fusion(X, P, track_fusion_index, track_index,radar_index)
            %航迹初始化
            if isrow(X)
                X = X';
            end
            obj.X = X(1:9);
            obj.P = P;
            obj.connection_status = 0;
            obj.track_fusion_index = track_fusion_index;
            obj.track_property = 0;
            obj.connection_table = ones(1,2) * -1;
            obj.track_quality = ones(1,2) * -1;
            if radar_index == 1
                obj.connection_table(1) = track_index;%分站航迹批号
                obj.track_type = 0;%与雷达1匹配上
                obj.track_quality(1) = 1;
            else
                obj.connection_table(2) = track_index;
                obj.track_type = 1;%与雷达1匹配上
                obj.track_quality(2) = 1;
            end
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

