% Author: Shen Kailun
% Date  : 2023-11-14
classdef Plot_Track
    %点迹
    properties
        X %状态向量
        Polar_X %极坐标状态向量
        connectionStatus %关联状态 0：未关联 1：关联
    end
    methods
        function obj = Plot_Track(X,Polar_X)
            %PLOT_TRACK 构造此类的实例
            %   此处显示详细说明
            obj.X = X;
            obj.connectionStatus = 0;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

