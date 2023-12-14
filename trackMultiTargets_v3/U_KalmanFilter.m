classdef U_KalmanFilter
    %KALMANFILTER ¿¨¶ûÂüÂË²¨Æ÷
    properties
        F %F ×´Ì¬×ªÒÆ¾ØÕó
        H %H ¹Û²â¾ØÕó
        Q %Q ¹ý³ÌÔëÉù¾ØÕó
        R %R Á¿²âÔëÉù¾ØÕó
        W
        nx
        k
    end
    methods
        function obj = U_KalmanFilter(F,H,Q,R)
            %F ×´Ì¬×ªÒÆ¾ØÕó H ¹Û²â¾ØÕó
            %Q ¹ý³ÌÔëÉù¾ØÕó R Á¿²âÔëÉù¾ØÕó
            obj.F = F;
            obj.H = H;
            obj.Q = Q;
            obj.R = R;
            obj.nx = 9;obj.k=-3;
            obj.W=zeros(1,2*obj.nx+1);%È¨Öµ
            obj.W(1)=obj.k/(obj.nx+obj.k);
            obj.W(2:1+2*obj.nx)=1/(2*(obj.nx+obj.k));
        end
        function [X_predict, P_predict, x_forecast_sigma] = U_KalmanPredict(obj,X, P)
            %º¯Êý¹¦ÄÜ£º ¿¨¶ûÂüÔ¤²â
            %X ×´Ì¬ÏòÁ¿
            %P Îó²îÐ­·½²î¾ØÕó
            X1=zeros(obj.nx,2*obj.nx+1);%²ÉÑùµã
            X1(:,1)=X;
            X1(:,2:1+obj.nx)=repmat(X,1,obj.nx)+chol((obj.nx+obj.k)*P,'lower');
            X1(:,2+obj.nx:1+2*obj.nx)=repmat(X,1,obj.nx)-chol((obj.nx+obj.k)*P,'lower');
            for j=1:2*obj.nx+1
                x_forecast_sigma(:,j) = obj.F * X1(:,j);        %Ô¤²âÖµ
            end
            X_predict=sum(obj.W.*x_forecast_sigma,2);%Ô¤²â×´Ì¬ÏòÁ¿
            P_predict=zeros(obj.nx,obj.nx);
            for j=1:2*obj.nx+1
               P_predict=P_predict+obj.W(j)*(x_forecast_sigma(:,j)-X_predict)*(x_forecast_sigma(:,j)-X_predict).';
            end
            P_predict=P_predict+obj.Q;%Ô¤²âÎó²îÐ­·½²î¾ØÕó
        end
        
        
        function [X_update, P_update] = U_KalmanUpdate(obj,X, P, x_forecast_sigma, Z)
            %º¯Êý¹¦ÄÜ£º ¿¨¶ûÂüÂË²¨
            z1=zeros(3,1);
            y_yuce=[];
            for j=1:2*obj.nx+1
                r = sqrt(x_forecast_sigma(1,j)^2+x_forecast_sigma(4,j)^2+x_forecast_sigma(7,j)^2);
                alpha = atan2(x_forecast_sigma(1,j),x_forecast_sigma(4,j));
                beta =atan(x_forecast_sigma(7,j)/sqrt(x_forecast_sigma(1,j)^2+x_forecast_sigma(4,j)^2));
                y_yuce = [y_yuce [r,alpha,beta]'];
                z1=z1+obj.W(j)*y_yuce(:,end);%Ô¤²âÁ¿²â
            end
            Pzz=0;
            for j=1:2*obj.nx+1
                Pzz=Pzz+obj.W(j)*(y_yuce(:,j)-z1)*(y_yuce(:,j)-z1).';%Ô¤²âÁ¿²âÎó²îÐ­·½²î¾ØÕó
            end
            Pzz=Pzz+obj.R;
            Pxz=zeros(obj.nx,1);
            for j=1:2*obj.nx+1
               Pxz=Pxz+obj.W(j)*(x_forecast_sigma(:,j)-X)*(y_yuce(:,j)-z1).';%Ô¤²âÁ¿²â ×´Ì¬ÏòÁ¿½»»¥Îó²îÐ­·½²î¾ØÕó
            end    
            K=Pxz/(Pzz);%ÔöÒæ
            Z(2) = Z(2)*pi/180;Z(3) = Z(3)*pi/180;
            X_update=X+K*(Z-z1);%ÂË²¨
            x=X_update(1);y=X_update(4);z=X_update(7);
            h = [x/(x^2 + y^2 + z^2)^(1/2),0,0,y/(x^2 + y^2 + z^2)^(1/2),0,0,z/(x^2 + y^2 + z^2)^(1/2),0,0;...
                1/(y^2*(x^2/y^2 + 1)),0,0,-x/(y^2*(x^2/y^2 + 1)),0,0,0,0,0;...
                -(x*z)/((z^2/(x^2 + y^2) + 1)*(x^2 + y^2)^(3/2)),0,0,-(y*z)/((z^2/(x^2 + y^2) + 1)*(x^2 + y^2)^(3/2)),0,0,1/((z^2/(x^2 + y^2) + 1)*(x^2 + y^2)^(1/2)),0,0];              
            P_update=P-K*Pzz*K';%ÂË²¨
%             P_update = (eye(length(X)) - K * h) * P * (eye(length(X)) - K * h)' + K * obj.R * K';
        end
        function [X_predict, P_predict, x_forecast_sigma] = U_KalmanPredict_specifiedT(obj,X, P, T)
            %º¯Êý¹¦ÄÜ£º ¿¨¶ûÂüÔ¤²â
            %X ×´Ì¬ÏòÁ¿
            %P Îó²îÐ­·½²î¾ØÕó
            if T >= 0
                %X ×´Ì¬ÏòÁ¿
                %P Îó²îÐ­·½²î¾ØÕó                
                F=[1 T 1/2*T^2 0 0 0 0 0 0 ;...
                    0 1 T 0 0 0 0 0 0;...
                    0 0 1 0 0 0 0 0 0;...
                    0 0 0 1 T 1/2*T^2 0 0 0;...
                    0 0 0 0 1 T 0 0 0;...
                    0 0 0 0 0 1 0 0 0;...
                    0 0 0 0 0 0 1 T T^2/2;...
                    0 0 0 0 0 0 0 1 T;...
                    0 0 0 0 0 0 0 0 1];%×´Ì¬×ªÒÆ¾ØÕó   

                X1=zeros(obj.nx,2*obj.nx+1);%²ÉÑùµã
                X1(:,1)=X;
                X1(:,2:1+obj.nx)=repmat(X,1,obj.nx)+chol((obj.nx+obj.k)*P,'lower');
                X1(:,2+obj.nx:1+2*obj.nx)=repmat(X,1,obj.nx)-chol((obj.nx+obj.k)*P,'lower');
                for j=1:2*obj.nx+1
                    x_forecast_sigma(:,j) = F * X1(:,j);        %Ô¤²âÖµ
                end
                X_predict=sum(obj.W.*x_forecast_sigma,2);%Ô¤²â×´Ì¬ÏòÁ¿
                P_predict=zeros(obj.nx,obj.nx);
                for j=1:2*obj.nx+1
                   P_predict=P_predict+obj.W(j)*(x_forecast_sigma(:,j)-X_predict)*(x_forecast_sigma(:,j)-X_predict).';
                end
                P_predict=P_predict+obj.Q;%Ô¤²âÎó²îÐ­·½²î¾ØÕó               
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
                X1=zeros(obj.nx,2*obj.nx+1);%²ÉÑùµã
                X1(:,1)=X;
                X1(:,2:1+obj.nx)=repmat(X,1,obj.nx)+chol((obj.nx+obj.k)*P,'lower');
                X1(:,2+obj.nx:1+2*obj.nx)=repmat(X,1,obj.nx)-chol((obj.nx+obj.k)*P,'lower');
                for j=1:2*obj.nx+1
                    x_forecast_sigma(:,j) = inv_F * X1(:,j);        %Ô¤²âÖµ
                end
                X_predict=sum(obj.W.*x_forecast_sigma,2);%Ô¤²â×´Ì¬ÏòÁ¿
                P_predict=zeros(obj.nx,obj.nx);
                for j=1:2*obj.nx+1
                   P_predict=P_predict+obj.W(j)*(x_forecast_sigma(:,j)-X_predict)*(x_forecast_sigma(:,j)-X_predict).';
                end
                P_predict=P_predict+obj.Q;%Ô¤²âÎó²îÐ­·½²î¾ØÕó                  
            end

        end        
    end
end

