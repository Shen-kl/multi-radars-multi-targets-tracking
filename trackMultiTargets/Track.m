% Author: Shen Kailun
% Date  : 2023-11-14
classdef Track
    %航迹 类
    properties
        X %状态向量
        P %误差协方差矩阵
        Polar_X %极坐标
        track_quality %航迹得分
        connection_status %关联状态 0：未关联 1：关联成功
        track_index %批号
        zone_index
        track_property %0：滤波 1：外推 2：终结
    end
    
    methods
        function obj = Track(X, P, track_index,zone_index,track_quality)
            %航迹初始化
            if isrow(X)
                X = X';
            end
            obj.X = X;
            obj.P = P;
            obj.track_quality = track_quality;
            obj.connection_status = 0;
            obj.zone_index = zone_index;
            obj.track_index = track_index;
            obj.track_property = 0;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

