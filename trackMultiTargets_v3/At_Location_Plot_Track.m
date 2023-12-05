% Author: Shen Kailun
% Date  : 2023-11-15
classdef At_Location_Plot_Track
    properties
        plot_track_num
        plot_track_index
    end
    
    methods
        function obj = At_Location_Plot_Track()
            %ATT_LOCATION_PLOT_TRACK 构造此类的实例
            %   此处显示详细说明
            obj.plot_track_num = 0;
            obj.plot_track_index = [];
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

