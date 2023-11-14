% Author: Shen Kailun
% Date  : 2023-11-14
classdef Track
    %航迹 类
    properties
        X %状态向量
        P %误差协方差矩阵
        score %航迹得分
        connectionStatus %关联状态 0：未关联 1：大波门 2：小波门
    end
    
    methods
        function obj = Track(X, P)
            %航迹初始化
            obj.X = X;
            obj.P = P;
            obj.score = 1;
            obj.connectionStatus = 0;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

