% Author: Shen Kailun
% Date  : 2023-11-14
classdef Plot_Track
    %点迹
    properties
        X %状态向量
        Polar_X %极坐标状态向量
        connection_status %关联状态 0：未关联 1：关联
        zone_index %分区区号
        plot_track_index %点迹标号
        inDoor %落入相关波们标志位
    end
    methods
        function obj = Plot_Track(Polar_X, plot_track_index)
            %PLOT_TRACK 构造此类的实例
            if isrow(Polar_X)
                Polar_X =Polar_X';
            end
            obj.Polar_X = Polar_X;
            X = zeros(3,1);
            [X(1),X(2),X(3)] = aer2enu(Polar_X(2),Polar_X(3),Polar_X(1));
            obj.X = X;
            obj.connection_status = 0;
            obj.plot_track_index = plot_track_index;
            obj.inDoor = 0;
        end
        
    end
end

