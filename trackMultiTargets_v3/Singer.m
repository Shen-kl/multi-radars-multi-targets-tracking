% Author: Shen Kailun
% Date  : 2023-12-1
classdef Singer
    %SINGER     
    properties
            Pmax_x
            Pmax_y
            Pmax_z
            P0_x
            P0_y
            P0_z
            amax_x
            amax_y
            amax_z
            alpha_x
            alpha_y
            alpha_z
            sigma2_x
            sigma2_y
            sigma2_z
            F %F ×´Ì¬×ªÒÆ¾ØÕó
            H %H ¹Û²â¾ØÕó
            Q %Q ¹ý³ÌÔëÉù¾ØÕó
            R %R Á¿²âÔëÉù¾ØÕó        
    end
    
    methods
        function obj = Singer(Pmax_x,Pmax_y,Pmax_z,...
                         P0_x, P0_y,P0_z,...
                         amax_x,amax_y,amax_z,...
                         alpha_x,alpha_y,alpha_z,...
                         T,R,H)
            obj.Pmax_x=Pmax_x; obj.P0_x=P0_x;obj.amax_x=amax_x;obj.alpha_x=alpha_x;obj.sigma2_x=amax_x^2/3*(1+4*Pmax_x-P0_x);
            obj.Pmax_y=Pmax_y; obj.P0_y=P0_y;obj.amax_y=amax_y;obj.alpha_y=alpha_y;obj.sigma2_y=amax_y^2/3*(1+4*Pmax_y-P0_y);
            obj.Pmax_z=Pmax_z; obj.P0_z=P0_z;obj.amax_z=amax_z;obj.alpha_z=alpha_z;obj.sigma2_z=amax_z^2/3*(1+4*Pmax_z-P0_z);

            obj.F=[1 T (alpha_x*T-1+exp(-alpha_x*T))/alpha_x^2 0 0 0 0 0 0;...
            0 1 (1-exp(-alpha_x*T))/alpha_x 0 0 0 0 0 0;...
            0 0 exp(-alpha_x*T) 0 0 0 0 0 0;...
            0 0 0 1 T (alpha_y*T-1+exp(-alpha_y*T))/alpha_y^2 0 0 0;...
            0 0 0 0 1 (1-exp(-alpha_y*T))/alpha_y 0 0 0;...
            0 0 0 0 0 exp(-alpha_y*T) 0 0 0;...
            0 0 0 0 0 0 1 T (alpha_z*T-1+exp(-alpha_z*T))/alpha_z^2;...
            0 0 0 0 0 0 0 1 (1-exp(-alpha_z*T))/alpha_z;...
            0 0 0 0 0 0 0 0 exp(-alpha_z*T)];%×´Ì¬×ªÒÆ¾ØÕó

            obj.R=R;%WµÄÐ­·½²î¾ØÕó 
            obj.H=H;%¹Û²â¾ØÕó
            q11_x=(1-exp(-2*alpha_x*T)+2*alpha_x*T+2*alpha_x^3*T^3/3-2*alpha_x^2*T^2-4*alpha_x*T*exp(-alpha_x*T))/(2*alpha_x^5);
            q12_x=(exp(-2*alpha_x*T)+1-2*exp(-alpha_x*T)+2*alpha_x*T*exp(-alpha_x*T)-2*alpha_x*T+alpha_x^2*T^2)/(2*alpha_x^4);
            q13_x=(1-exp(-2*alpha_x*T)-2*alpha_x*T*exp(-alpha_x*T))/(2*alpha_x^3);
            q21_x=q12_x;
            q22_x=(4*exp(-alpha_x*T)-3-exp(-2*alpha_x*T)+2*alpha_x*T)/(2*alpha_x^3);
            q23_x=(exp(-2*alpha_x*T)+1-2*exp(-alpha_x*T))/(2*alpha_x^2);
            q31_x=q13_x;
            q32_x=q23_x;
            q33_x=(1-exp(-2*alpha_x*T))/(2*alpha_x);
            Q_x=2*alpha_x*[q11_x q12_x q13_x;q21_x q22_x q23_x;q31_x q32_x q33_x];

            q11_y=(1-exp(-2*alpha_y*T)+2*alpha_y*T+2*alpha_y^3*T^3/3-2*alpha_y^2*T^2-4*alpha_y*T*exp(-alpha_y*T))/(2*alpha_y^5);
            q12_y=(exp(-2*alpha_y*T)+1-2*exp(-alpha_y*T)+2*alpha_y*T*exp(-alpha_y*T)-2*alpha_y*T+alpha_y^2*T^2)/(2*alpha_y^4);
            q13_y=(1-exp(-2*alpha_y*T)-2*alpha_y*T*exp(-alpha_y*T))/(2*alpha_y^3);
            q21_y=q12_x;
            q22_y=(4*exp(-alpha_y*T)-3-exp(-2*alpha_y*T)+2*alpha_y*T)/(2*alpha_y^3);
            q23_y=(exp(-2*alpha_y*T)+1-2*exp(-alpha_y*T))/(2*alpha_y^2);
            q31_y=q13_x;
            q32_y=q23_x;
            q33_y=(1-exp(-2*alpha_y*T))/(2*alpha_y);
            Q_y=2*alpha_y*[q11_y q12_y q13_y;q21_y q22_y q23_y;q31_y q32_y q33_y];


            q11_z=(1-exp(-2*alpha_z*T)+2*alpha_z*T+2*alpha_z^3*T^3/3-2*alpha_z^2*T^2-4*alpha_z*T*exp(-alpha_z*T))/(2*alpha_z^5);
            q12_z=(exp(-2*alpha_z*T)+1-2*exp(-alpha_z*T)+2*alpha_z*T*exp(-alpha_z*T)-2*alpha_z*T+alpha_z^2*T^2)/(2*alpha_z^4);
            q13_z=(1-exp(-2*alpha_z*T)-2*alpha_z*T*exp(-alpha_z*T))/(2*alpha_z^3);
            q21_z=q12_x;
            q22_z=(4*exp(-alpha_z*T)-3-exp(-2*alpha_z*T)+2*alpha_z*T)/(2*alpha_z^3);
            q23_z=(exp(-2*alpha_z*T)+1-2*exp(-alpha_z*T))/(2*alpha_z^2);
            q31_z=q13_x;
            q32_z=q23_x;
            q33_z=(1-exp(-2*alpha_z*T))/(2*alpha_z);
            Q_z=2*alpha_z*[q11_z q12_z q13_z;q21_z q22_z q23_z;q31_z q32_z q33_z];

            obj.Q=[Q_x zeros(3,6);zeros(3,3) Q_y zeros(3,3);zeros(3,6) Q_z];%ÏµÍ³ÔëÉù¾ØÕó
        end
        
        function [X_predict, P_predict] = SingerPredict(obj,X, P)
            %º¯Êý¹¦ÄÜ£º ¿¨¶ûÂüÔ¤²â
            %X ×´Ì¬ÏòÁ¿
            %P Îó²îÐ­·½²î¾ØÕó
            X_predict = obj.F * X;
            P_predict = obj.F * P * obj.F' + obj.Q;
        end
        function [X_update, P_update] = SingerUpdate(obj,X, P, Z)
            %º¯Êý¹¦ÄÜ£º ¿¨¶ûÂüÂË²¨
            Z_pre= obj.H * X;
            K = P *obj.H' /(obj.H * P * obj.H' + obj.R);%¼ÆËãÔöÒæ¾ØÕó
            X_update = X + K * (Z - Z_pre);
            P_update = (eye(length(X)) - K * obj.H) * P * (eye(length(X)) - K * obj.H)' + K * obj.R * K';
        end
        function [X_predict, P_predict] = SingerPredict_specifiedT(obj,X, P, T)
            %º¯Êý¹¦ÄÜ£º ¿¨¶ûÂüÔ¤²â
            %X ×´Ì¬ÏòÁ¿
            %P Îó²îÐ­·½²î¾ØÕó
            if T >= 0
            F=[1 T (obj.alpha_x*T-1+exp(-obj.alpha_x*T))/obj.alpha_x^2 0 0 0 0 0 0;...
                0 1 (1-exp(-obj.alpha_x*T))/obj.alpha_x 0 0 0 0 0 0;...
                0 0 exp(-obj.alpha_x*T) 0 0 0 0 0 0;...
                0 0 0 1 T (obj.alpha_y*T-1+exp(-obj.alpha_y*T))/obj.alpha_y^2 0 0 0;...
                0 0 0 0 1 (1-exp(-obj.alpha_y*T))/obj.alpha_y 0 0 0;...
                0 0 0 0 0 exp(-obj.alpha_y*T) 0 0 0;...
                0 0 0 0 0 0 1 T (obj.alpha_z*T-1+exp(-obj.alpha_z*T))/obj.alpha_z^2;...
                0 0 0 0 0 0 0 1 (1-exp(-obj.alpha_z*T))/obj.alpha_z;...
                0 0 0 0 0 0 0 0 exp(-obj.alpha_z*T)];%×´Ì¬×ªÒÆ¾ØÕó
                X_predict = F * X;
                P_predict = F * P * F' + obj.Q;                 
            else
                T = abs(T);
                F=[1 T (obj.alpha_x*T-1+exp(-obj.alpha_x*T))/obj.alpha_x^2 0 0 0 0 0 0;...
                    0 1 (1-exp(-obj.alpha_x*T))/obj.alpha_x 0 0 0 0 0 0;...
                    0 0 exp(-obj.alpha_x*T) 0 0 0 0 0 0;...
                    0 0 0 1 T (obj.alpha_y*T-1+exp(-obj.alpha_y*T))/obj.alpha_y^2 0 0 0;...
                    0 0 0 0 1 (1-exp(-obj.alpha_y*T))/obj.alpha_y 0 0 0;...
                    0 0 0 0 0 exp(-obj.alpha_y*T) 0 0 0;...
                    0 0 0 0 0 0 1 T (obj.alpha_z*T-1+exp(-obj.alpha_z*T))/obj.alpha_z^2;...
                    0 0 0 0 0 0 0 1 (1-exp(-obj.alpha_z*T))/obj.alpha_z;...
                    0 0 0 0 0 0 0 0 exp(-obj.alpha_z*T)];%×´Ì¬×ªÒÆ¾ØÕó      
                inv_F = inv(F);
                X_predict = inv_F * X;
                P_predict = inv_F * P * inv_F' + obj.Q;                
            end
        end
    end
end

