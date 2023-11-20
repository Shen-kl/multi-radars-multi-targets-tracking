% Author: Shen Kailun
% Date  : 2023-11-19
classdef Fusion
    %FUSSION 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        tracker_method %'跟踪方法'
        tracker %跟踪器
        T %帧周期
        track_fusion_set %航迹集合
        track_fusion_num %航迹数量
        Chi_large%对应2维 门概率质量PG=1
        Chi_small%对应2维 门概率质量PG 
        radar_track_set %各分站上报数据
        system_frame_index%系统时间
    end
    
    methods
        function obj = Fusion(tracker_method, T, Chi_large, Chi_small)
            obj.tracker_method =  tracker_method;%'跟踪方法'
            obj.T = T;%帧周期
            obj.Chi_large = Chi_large;%对应2维 门概率质量PG=1
            obj.Chi_small = Chi_small;%对应2维 门概率质量PG  
            obj.track_fusion_num = 0;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
        
        
        function [x,value]=find_track(obj,num)
        %根据航迹号找对应的航迹
        %input num 想要找的本地航迹的航迹编号
        %output x 该本地航迹在track_header_set中数组的序号
        %output value 1:成功找到 0：未找到
            value=0;
            if ~isempty(obj.radar_track_set)
                for x=1:size(obj.radar_track_set,2)
                    if obj.radar_track_set(x).track_report_index==num
                        value=1;
                        break;
                    end
                end
            else
                error('radar_track_set is empty!');
            end
        end   
        
        function [D,flag] = Related_gate_track2track(obj,track_fusion_index,radar_track_index,Chi)
            %相关波门判断
            %input track_fusion_index 此条本地航迹在数组中的序列号
            %input radar_track_fusion_index 此条系统航迹在数组中的序列号    
            %input Chi 门限常数

            %output D 统计距离
            %output flag 1 :相关 0：不相关

            S=obj.tracker.H*(obj.track_fusion_set(track_fusion_index).P + obj.radar_track_set(radar_track_index).P)*obj.tracker.H'; %计算椭圆跟踪门的面积 
        %     ellipse_Volume=pi*Chi*sqrt(det(S));                             
            d=obj.track_fusion_set(track_fusion_index).X(1:3:9,end)-obj.radar_track_set(radar_track_index).X(1:3:9,end);%预测状态向量与观测值的残差  

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
        
        function [costMatrix,flagMatrix] = calCostMatrix(obj,track_fusion_set, radar_track_index_set)
            %计算代价矩阵 即欧氏距离矩阵
            %矩阵沿着列方向为航迹 行方向为观测点迹
            costMatrix = zeros(length(track_fusion_set),length(radar_track_index_set));
            flagMatrix = zeros(length(track_fusion_set),length(radar_track_index_set));
            for i =1 : length(track_fusion_set)
                for j = 1 : length(radar_track_index_set)
                    [D,flag] = obj.Related_gate_track2track(track_fusion_set(i),radar_track_index_set(j),obj.Chi_large);
                    costMatrix(i,j) = D;
                    flagMatrix(i,j) = flag;
                end
            end      
        end
        
        function [fusion_X, fusion_P] = convexCombinationFusion(obj, system_X, system_P, radar_X, radar_P)
            %凸组合融合算法
            W1 = radar_P * inv(system_P + radar_P);
            I9 = eye(9);
            W2 = I9 - W1;
            fusion_X = W1 * system_X + W2 * radar_X(1:9);
            fusion_P = system_P * inv(system_P + radar_P) * radar_P;
        end
    end

end

