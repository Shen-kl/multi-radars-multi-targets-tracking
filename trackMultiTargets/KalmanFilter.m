% Author: Shen Kailun
% Date  : 2023-11-14
classdef KalmanFilter
    %KALMANFILTER 卡尔曼滤波器
    properties
        F %F 状态转移矩阵
        H %H 观测矩阵
        Q %Q 过程噪声矩阵
        R %R 量测噪声矩阵
    end
    methods
        function obj = KalmanFilter(F,H,Q,R)
            %F 状态转移矩阵 H 观测矩阵
            %Q 过程噪声矩阵 R 量测噪声矩阵
            obj.F = F;
            obj.H = H;
            obj.Q = Q;
            obj.R = R;
        end
        function [X_predict, P_predict] = KalmanPredict(obj,X, P)
            %函数功能： 卡尔曼预测
            %X 状态向量
            %P 误差协方差矩阵
            X_predict = obj.F * X;
            P_predict = obj.F * P + obj.F' + obj.Q;
        end
        function [X_update, P_update] = KalmanUpdate(obj,X, P, Z)
            %函数功能： 卡尔曼滤波
            Z_pre= obj.H * X;
            K = P *obj.H' /(obj.H * P * obj.H' + obj.R);%计算增益矩阵
            X_update = X + K * (Z - Z_pre);
            P_update = (eye(length(X)) - K * obj.H) * P * (eye(length(X)) - K * obj.H)' + K * obj.R * K';
        end
    end
end

