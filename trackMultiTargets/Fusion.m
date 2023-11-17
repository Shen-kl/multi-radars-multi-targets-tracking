classdef Fusion
    %FUSSION 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        tracker_method %'跟踪方法'
        tracker %跟踪器
        T %帧周期
        track_set %航迹集合
        track_num %航迹数量
        Chi_large%对应2维 门概率质量PG=1
        Chi_small%对应2维 门概率质量PG         
    end
    
    methods
        function obj = Fusion(inputArg1,inputArg2)
            %FUSSION 构造此类的实例
            %   此处显示详细说明
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

