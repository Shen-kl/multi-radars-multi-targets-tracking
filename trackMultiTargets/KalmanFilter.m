% Author: Shen Kailun
% Date  : 2023-11-14
classdef KalmanFilter
    %KALMANFILTER ¿¨¶ûÂüÂË²¨Æ÷
    properties
        F %F ×´Ì¬×ªÒÆ¾ØÕó
        H %H ¹Û²â¾ØÕó
        Q %Q ¹ý³ÌÔëÉù¾ØÕó
        R %R Á¿²âÔëÉù¾ØÕó
    end
    methods
        function obj = KalmanFilter(F,H,Q,R)
            %F ×´Ì¬×ªÒÆ¾ØÕó H ¹Û²â¾ØÕó
            %Q ¹ý³ÌÔëÉù¾ØÕó R Á¿²âÔëÉù¾ØÕó
            obj.F = F;
            obj.H = H;
            obj.Q = Q;
            obj.R = R;
        end
        function [X_predict, P_predict] = KalmanPredict(obj,X, P)
            %º¯Êý¹¦ÄÜ£º ¿¨¶ûÂüÔ¤²â
            %X ×´Ì¬ÏòÁ¿
            %P Îó²îÐ­·½²î¾ØÕó
            X_predict = obj.F * X;
            P_predict = obj.F * P + obj.F' + obj.Q;
        end
        function [X_update, P_update] = KalmanUpdate(obj,X, P, Z)
            %º¯Êý¹¦ÄÜ£º ¿¨¶ûÂüÂË²¨
            Z_pre= obj.H * X;
            K = P *obj.H' /(obj.H * P * obj.H' + obj.R);%¼ÆËãÔöÒæ¾ØÕó
            X_update = X + K * (Z - Z_pre);
            P_update = (eye(length(X)) - K * obj.H) * P * (eye(length(X)) - K * obj.H)' + K * obj.R * K';
        end
        function [X_predict, P_predict] = KalmanPredict_specifiedT(obj,X, P, T)
            %º¯Êý¹¦ÄÜ£º ¿¨¶ûÂüÔ¤²â
            %X ×´Ì¬ÏòÁ¿
            %P Îó²îÐ­·½²î¾ØÕó
            if T >= 0
                F=[1 T 1/2*T^2 0 0 0 0 0 0 ;...
                    0 1 T 0 0 0 0 0 0;...
                    0 0 1 0 0 0 0 0 0;...
                    0 0 0 1 T 1/2*T^2 0 0 0;...
                    0 0 0 0 1 T 0 0 0;...
                    0 0 0 0 0 1 0 0 0;...
                    0 0 0 0 0 0 1 T T^2/2;...
                    0 0 0 0 0 0 0 1 T;...
                    0 0 0 0 0 0 0 0 1];%×´Ì¬×ªÒÆ¾ØÕó   
                X_predict = F * X;
                P_predict = F * P + F' + obj.Q;                 
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
                    0 0 0 0 0 0 0 0 1];%×´Ì¬×ªÒÆ¾ØÕó         
                inv_F = inv(F);
                X_predict = inv_F * X;
                P_predict = inv_F * P + inv_F' + obj.Q;                
            end

        end        
    end
end

