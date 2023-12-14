classdef ExtentedKalmanFilter
    %ExtentedKalmanFilter 扩展卡尔曼滤�?
    properties
        F %F 状�?�转移矩�?
        jacobian_F %jacobian_F 状�?�转移矩阵的雅可比矩�?
%         H %H 观测矩阵
%         jacobian_H %jacobian_H 观测矩阵的雅可比矩阵
        Q %Q 过程噪声矩阵
        R %R 量测噪声矩阵
        H
    end

    methods
        function obj = ExtentedKalmanFilter(F,jacobian_F,Q,R,H)
            %F 状�?�转移矩�? H 观测矩阵
            %Q 过程噪声矩阵 R 量测噪声矩阵
            obj.F = F;
            obj.jacobian_F = jacobian_F;
            obj.Q = Q;
            obj.R = R;
            obj.H = H;
        end

        function [X_predict, P_predict] = ExtentedKalmanPredict(obj,X, P)
            %函数功能�? 扩展卡尔曼预�?
            %X 状�?�向�?
            %P 误差协方差矩�?
            X_predict = obj.F * X;
%             obj.jacobian_F = double(subs(obj.jacobian_F,{x,x_1,x_2,y,y_1,y_2,z,z_1,z_2}, ...
%                 {X(1),X(2),X(3),X(4),X(5),X(6),X(7),X(8),X(8)}));
            P_predict = obj.jacobian_F * P * obj.jacobian_F' + obj.Q;
        end

        function [X_update, P_update] = ExtentedKalmanUpdate(obj,X, P, Z)
            %函数功能�? 扩展卡尔曼滤�?
%             Z = double(subs(obj.H,{'x','y','z'},{Z(1),Z(2),Z(3)}));
%             Z = mearsure_matrix(Z(1),Z(2),Z(3));
%             Z_pre= double(subs(obj.H,{'x','y','z'},{X(1),X(4),X(7)}));
            Z(2) = Z(2)*pi/180;
            Z(3) = Z(3)*pi/180;
            Z_pre= mearsure_matrix(X(1),X(4),X(7));
%             obj.jacobian_H = double(subs(obj.jacobian_H,{'x','x_1','x_2','y','y_1','y_2','z','z_1','z_2'}, ...
%                 {X(1),X(2),X(3),X(4),X(5),X(6),X(7),X(8),X(9)}));
            jaco_H = mearsure_jacobian(X(1),X(4),X(7));
            S = jaco_H * P * jaco_H' + obj.R;%新息协方�?
            K = P *jaco_H' / S;%计算增益矩阵
            X_update = X + K * (Z - Z_pre);
            P_update = (eye(length(X)) - K * jaco_H) * P * (eye(length(X)) - K * jaco_H)' + K * obj.R * K';
%             P_update = (eye(length(X)) - K * jaco_H) * P;
        end

        function [X_predict, P_predict] = ExtentedKalmanPredict_specifiedT(obj,X, P, T)
            %函数功能�? 卡尔曼预�?
            %X 状�?�向�?
            %P 误差协方差矩�?
            if T >= 0
                F=[1 T 1/2*T^2 0 0 0 0 0 0 ;...
                    0 1 T 0 0 0 0 0 0;...
                    0 0 1 0 0 0 0 0 0;...
                    0 0 0 1 T 1/2*T^2 0 0 0;...
                    0 0 0 0 1 T 0 0 0;...
                    0 0 0 0 0 1 0 0 0;...
                    0 0 0 0 0 0 1 T T^2/2;...
                    0 0 0 0 0 0 0 1 T;...
                    0 0 0 0 0 0 0 0 1];%状�?�转移矩�?   
                X_predict = F * X;
                P_predict = F * P * F' + obj.Q;                 
            else
                T = abs(T);
                F=[1 T 1/2*T^2 0 0 0 0 0 0 ;...
                    0 1 T 0 0 0 0 0 0;...
                    0 0 1 0 0 0 0 0 0;...
                    0 0 0 1 T 1/2*T^2 0 0 0;...
                    0 0 0 0 1 T 0 0 0;...
                    0 0 0 0 0 1 0 0 0;...
                    0 0 0 0 0 0 1 T T^2/2;...
                    0 0 0 0 0 0 0 1 T;...
                    0 0 0 0 0 0 0 0 1];%状�?�转移矩�?         
                inv_F = inv(F);
                X_predict = inv_F * X;
                P_predict = inv_F * P * inv_F' + obj.Q;                
            end
        end
    end
end