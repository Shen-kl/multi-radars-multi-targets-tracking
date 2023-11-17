% Author: Shen Kailun
% Date  : 2023-11-14
classdef Track_Header
    %航迹 类
    properties
        header_index
        X %状态向量
        Polar_X %极坐标 
        track_quality %航迹得分
        connection_status %关联状态 0：未关联 1：关联
        likelihoodProbability %航迹似然比
        zone_index %分区号
        inDoor %落入波门标志位
    end
    
    methods
        function obj = Track_Header(header_index,X,Polar_X,...
                zone_index,likelihoodProbability)
            %航迹初始化
            obj.header_index =  header_index;
            if isrow(X)
                X =X';
            end            
            obj.X = X;
            if isrow(Polar_X)
                Polar_X =Polar_X';
            end            
            obj.Polar_X = Polar_X;
            obj.zone_index = zone_index;
            obj.likelihoodProbability = likelihoodProbability;
            obj.track_quality = 1;
            obj.connection_status = 0;
            obj.inDoor = 0;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

