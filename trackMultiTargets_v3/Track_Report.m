% Author: Shen Kailun
% Date  : 2023-11-17
classdef Track_Report
    %上报航迹
    properties
        X %状态向量
        X_predict %预测状态向量
        Polar_X %极坐标状态向量
        connection_status %关联状态 0：未关联 1：关联
        track_report_index %批号
        inDoor %落入相关波们标志位
        P %误差协方差矩阵
        P_predict %预测误差协方差矩阵
        radar_index %雷达标号
    end
    
    methods
        function obj = Track_Report(X, X_predict, P, P_predict, track_report_index,radar_index)
            obj.X = X;
            obj.X_predict = X_predict;
            obj.P = P;
            obj.P_predict = P_predict;
            Polar_X = zeros(3,1);
            [Polar_X(2),Polar_X(3),Polar_X(1)] = enu2aer(X(1),X(2),X(3));
            obj.Polar_X = Polar_X;
            obj.connection_status = 0;
            obj.inDoor = 0;
            obj.track_report_index = track_report_index;
            obj.radar_index =  radar_index;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

