%% 第四种场景 8字形
clear all;
clc;
close all;
g = 9.8;
T = 0.2;%数据率
W=[50,0.2,0.2];%观测误差
H=[1,0,0,0,0,0,0,0,0;...
    0,0,0,1,0,0,0,0,0;...
    0,0,0,0,0,0,1,0,0];
% disRangle = [10e3 : 1 : 15e3];
Ttotal = 360; %总帧数
scene = 4;


STR_maneuver = struct(...
    'begin_index',[],...
     'end_index',[],...
     'type',[],...
     'acc_x',[],...
     'acc_y',[],...
     'acc_z',[],...
     'angular_velocity',[],...
     'angular_velocity_x',[],...
     'angular_velocity_y',[],...
     'angular_velocity_z',[],...
     'F',[],...
     'process_noise',[]...
    ); 
    
manuver_time_index = [2,25,165,200,340,Ttotal];
maneuver_types_set = [{'CV'},{'HCT'},{'CV'},{'HCT'},{'CV'}]; %Horizontal coordinated turn(HCT) 3D space turn(3DTC)

% index 1
STR_maneuver(1).type = 'CV';              

F_CV = [1, T, 0;...
        0, 1, 0;...
        0, 0, 0];              
STR_maneuver(1).F = [F_CV,zeros(3,6);zeros(3,3),F_CV,zeros(3,3);zeros(3,6),F_CV]; 
STR_maneuver(1).process_noise_accelerated_velocity = 1;                 
STR_maneuver(1).process_noise = @(process_noise_accelerated_velocity)...
    [[0.5*T^2*normrnd(0,(process_noise_accelerated_velocity));
    T*normrnd(0,(process_noise_accelerated_velocity));
    1*normrnd(0,process_noise_accelerated_velocity)];
    [0.5*T^2*normrnd(0,(process_noise_accelerated_velocity));
    T*normrnd(0,(process_noise_accelerated_velocity));
    1*normrnd(0,process_noise_accelerated_velocity)];
    [0.5*T^2*normrnd(0,(process_noise_accelerated_velocity));
    T*normrnd(0,(process_noise_accelerated_velocity));
    1*normrnd(0,process_noise_accelerated_velocity)]]; 
% index 2
STR_maneuver(2).type = 'HCT';
STR_maneuver(2).angular_velocity = 10;
angular_velocity = STR_maneuver(2).angular_velocity;
F_HCT = [1 , sin(angular_velocity*T*pi/180)/(angular_velocity*pi/180), 0 , 0,(cos(angular_velocity*T*pi/180)-1)/(angular_velocity*pi/180), 0;...
        0 , cos(angular_velocity*T*pi/180), 0 ,0, -sin(angular_velocity*T*pi/180), 0;...
        0, -(angular_velocity*pi/180)*sin(angular_velocity*T*pi/180),0,0,-(angular_velocity*pi/180)*cos(angular_velocity*T*pi/180),0;...
        0,  -(cos(angular_velocity*T*pi/180)-1)/(angular_velocity*pi/180),0,1 ,sin(angular_velocity*T*pi/180)/(angular_velocity*pi/180),0;...
        0, sin(angular_velocity*T*pi/180) , 0 ,0 , cos(angular_velocity*T*pi/180),0;...
        0,(angular_velocity*pi/180)*cos(angular_velocity*T*pi/180), 0,0, -(angular_velocity*pi/180)*sin(angular_velocity*T*pi/180),0];
F_CV = [1, T, 0;...
        0, 1, 0;...
        0, 0, 0];
STR_maneuver(2).F = [F_HCT,zeros(6,3);...
                            zeros(3,6), F_CV];
STR_maneuver(2).process_noise_angular = 0.1*pi/180;          
STR_maneuver(2).process_noise_accelerated_velocity = 0.5;          
STR_maneuver(2).process_noise = @(process_noise_angular,process_noise_accelerated_velocity)...
    [[(angular_velocity/180*pi*T-sin(angular_velocity/180*pi*T))/(angular_velocity/180*pi)^3 * normrnd(0,(process_noise_angular));...
    (1-cos(angular_velocity/180*pi*T))/(angular_velocity/180*pi)^2* normrnd(0,(process_noise_angular));...
    sin(angular_velocity/180*pi*T)/(angular_velocity/180*pi) * normrnd(0,(process_noise_angular))];
    [(angular_velocity/180*pi*T-sin(angular_velocity/180*pi*T))/(angular_velocity/180*pi)^3 * normrnd(0,(process_noise_angular));...
    (1-cos(angular_velocity/180*pi*T))/(angular_velocity/180*pi)^2* normrnd(0,(process_noise_angular));...
    sin(angular_velocity/180*pi*T)/(angular_velocity/180*pi) * normrnd(0,(process_noise_angular))];
    [0.5*T^2*normrnd(0,(process_noise_accelerated_velocity));
    T*normrnd(0,(process_noise_accelerated_velocity));
    0]];
% index 3
STR_maneuver(3).type = 'CV';              

F_CV = [1, T, 0;...
        0, 1, 0;...
        0, 0, 0];              
STR_maneuver(3).F = [F_CV,zeros(3,6);zeros(3,3),F_CV,zeros(3,3);zeros(3,6),F_CV]; 
STR_maneuver(3).process_noise_accelerated_velocity = 1;                 
STR_maneuver(3).process_noise = @(process_noise_accelerated_velocity)...
    [[0.5*T^2*normrnd(0,(process_noise_accelerated_velocity));
    T*normrnd(0,(process_noise_accelerated_velocity));
    1*normrnd(0,process_noise_accelerated_velocity)];
    [0.5*T^2*normrnd(0,(process_noise_accelerated_velocity));
    T*normrnd(0,(process_noise_accelerated_velocity));
    1*normrnd(0,process_noise_accelerated_velocity)];
    [0.5*T^2*normrnd(0,(process_noise_accelerated_velocity));
    T*normrnd(0,(process_noise_accelerated_velocity));
    1*normrnd(0,process_noise_accelerated_velocity)]]; 
% index 4
STR_maneuver(4).type = 'HCT';
STR_maneuver(4).angular_velocity = -10;
angular_velocity = STR_maneuver(4).angular_velocity;
F_HCT = [1 , sin(angular_velocity*T*pi/180)/(angular_velocity*pi/180), 0 , 0,(cos(angular_velocity*T*pi/180)-1)/(angular_velocity*pi/180), 0;...
        0 , cos(angular_velocity*T*pi/180), 0 ,0, -sin(angular_velocity*T*pi/180), 0;...
        0, -(angular_velocity*pi/180)*sin(angular_velocity*T*pi/180),0,0,-(angular_velocity*pi/180)*cos(angular_velocity*T*pi/180),0;...
        0,  -(cos(angular_velocity*T*pi/180)-1)/(angular_velocity*pi/180),0,1 ,sin(angular_velocity*T*pi/180)/(angular_velocity*pi/180),0;...
        0, sin(angular_velocity*T*pi/180) , 0 ,0 , cos(angular_velocity*T*pi/180),0;...
        0,(angular_velocity*pi/180)*cos(angular_velocity*T*pi/180), 0,0, -(angular_velocity*pi/180)*sin(angular_velocity*T*pi/180),0];
F_CV = [1, T, 0;...
        0, 1, 0;...
        0, 0, 0];
STR_maneuver(4).F = [F_HCT,zeros(6,3);...
                            zeros(3,6), F_CV];
STR_maneuver(4).process_noise_angular = 0.1*pi/180;          
STR_maneuver(4).process_noise_accelerated_velocity = 1;          
STR_maneuver(4).process_noise = @(process_noise_angular,process_noise_accelerated_velocity)...
    [[(angular_velocity/180*pi*T-sin(angular_velocity/180*pi*T))/(angular_velocity/180*pi)^3 * normrnd(0,(process_noise_angular));...
    (1-cos(angular_velocity/180*pi*T))/(angular_velocity/180*pi)^2* normrnd(0,(process_noise_angular));...
    sin(angular_velocity/180*pi*T)/(angular_velocity/180*pi) * normrnd(0,(process_noise_angular))];
    [(angular_velocity/180*pi*T-sin(angular_velocity/180*pi*T))/(angular_velocity/180*pi)^3 * normrnd(0,(process_noise_angular));...
    (1-cos(angular_velocity/180*pi*T))/(angular_velocity/180*pi)^2* normrnd(0,(process_noise_angular));...
    sin(angular_velocity/180*pi*T)/(angular_velocity/180*pi) * normrnd(0,(process_noise_angular))];
    [0.5*T^2*normrnd(0,(process_noise_accelerated_velocity));
    T*normrnd(0,(process_noise_accelerated_velocity));
    0]];
% index 5
STR_maneuver(5).type = 'CV';              

F_CV = [1, T, 0;...
        0, 1, 0;...
        0, 0, 0];              
STR_maneuver(5).F = [F_CV,zeros(3,6);zeros(3,3),F_CV,zeros(3,3);zeros(3,6),F_CV]; 
STR_maneuver(5).process_noise_accelerated_velocity = 1;                 
STR_maneuver(5).process_noise = @(process_noise_accelerated_velocity)...
    [[0.5*T^2*normrnd(0,(process_noise_accelerated_velocity));
    T*normrnd(0,(process_noise_accelerated_velocity));
    1*normrnd(0,process_noise_accelerated_velocity)];
    [0.5*T^2*normrnd(0,(process_noise_accelerated_velocity));
    T*normrnd(0,(process_noise_accelerated_velocity));
    1*normrnd(0,process_noise_accelerated_velocity)];
    [0.5*T^2*normrnd(0,(process_noise_accelerated_velocity));
    T*normrnd(0,(process_noise_accelerated_velocity));
    1*normrnd(0,process_noise_accelerated_velocity)]]; 

aziAngle(1) = 50;
eleAngle(1) = 20;

R1(1) = 1.6e3;
vx = 10;
vy = -20;
vz = 10;
ax = 0;
ay = 0;
az = 0;


x(:,1) =[R1(1)*sin(aziAngle(1)*pi/180)*cos(eleAngle(1)*pi/180);vx;ax;R1(1)*cos(aziAngle(1)*pi/180)*cos(eleAngle(1)*pi/180);vy;ay;R1(1)*sin(eleAngle(1)*pi/180);vz;az];
xf = x; 
xlog{1} = x;
[measlog{1}] = H*xf;
result = [sqrt(x(1,1)^2+x(4,1)^2+x(7,1)^2);...
        (atan(x(1,1)/x(4,1)));...
        (atan(x(7,1)/sqrt(x(1,1)^2 + x(4,1)^2)))];
[measlog_polar{1}] = result;
    
for index = 1 : length(manuver_time_index) - 1
    for t = manuver_time_index(index) : manuver_time_index(index + 1)
        if strcmp(STR_maneuver(index).type ,'3DCT')
            xtemp =xlog{t-1};
            w = sqrt(xtemp(3)^2+xtemp(6)^2+xtemp(9)^2)/sqrt(xtemp(2)^2+xtemp(5)^2+xtemp(8)^2);
            
            process_noise = STR_maneuver(index).process_noise(w,STR_maneuver(index).process_noise_angular); 
            xlog{t-1} = xtemp;
            if w~=0               
                xlog{t} =STR_maneuver(index).F(w)*xlog{t-1} + 1 * process_noise;
            else
                F_CA = [1, T, 0.5*T^2;...
                        0, 1, T;...
                        0, 0, 1];                     
                F = [F_CA,zeros(3,6);
                    zeros(3,3),F_CA,zeros(3,3);
                    zeros(3,6),F_CA];
                xlog{t} = F*xlog{t-1} + 10 * process_noise;
            end                 
        elseif strcmp(STR_maneuver(index).type ,'CV')
            process_noise = STR_maneuver(index).process_noise(STR_maneuver(index).process_noise_accelerated_velocity); 
            xlog{t} =STR_maneuver(index).F*xlog{t-1} + 1 * process_noise  ;
        elseif strcmp(STR_maneuver(index).type ,'CA')
            process_noise = STR_maneuver(index).process_noise(STR_maneuver(index).process_noise_accelerated_velocity); 
            xlog{t} =STR_maneuver(index).F*xlog{t-1} + 1 * process_noise  ;    
        elseif strcmp(STR_maneuver(index).type ,'HCT')
            process_noise = STR_maneuver(index).process_noise(STR_maneuver(index).process_noise_angular,STR_maneuver(index).process_noise_accelerated_velocity); 
            xlog{t} =STR_maneuver(index).F*xlog{t-1} + 1 * process_noise  ;                  
        end
         x = xlog{t};
         if x(7) < 1e2
            STR_maneuver(index).type = 'HCT';
            STR_maneuver(index).angular_velocity = 8;
            angular_velocity = STR_maneuver(index).angular_velocity;
            F_HCT = [1 , sin(angular_velocity*T*pi/180)/(angular_velocity*pi/180), 0 , 0,(cos(angular_velocity*T*pi/180)-1)/(angular_velocity*pi/180), 0;...
                    0 , cos(angular_velocity*T*pi/180), 0 ,0, -sin(angular_velocity*T*pi/180), 0;...
                    0, -(angular_velocity*pi/180)*sin(angular_velocity*T*pi/180),0,0,-(angular_velocity*pi/180)*cos(angular_velocity*T*pi/180),0;...
                    0,  -(cos(angular_velocity*T*pi/180)-1)/(angular_velocity*pi/180),0,1 ,sin(angular_velocity*T*pi/180)/(angular_velocity*pi/180),0;...
                    0, sin(angular_velocity*T*pi/180) , 0 ,0 , cos(angular_velocity*T*pi/180),0;...
                    0,(angular_velocity*pi/180)*cos(angular_velocity*T*pi/180), 0,0, -(angular_velocity*pi/180)*sin(angular_velocity*T*pi/180),0];
            F_CA = [1, T, 0.5*T^2;...
                    0, 0, 0;...
                    0, 0, 0];
            STR_maneuver(index).F = [F_HCT,zeros(6,3);...
                                        zeros(3,6), F_CA]; 
            while 1 
                xlog{t-1}(9) = 2*g;   
                if xlog{t-1}(9) ~= 0 
                    break;
                end
            end
            STR_maneuver(index).process_noise_angular = 0.05*pi/180;          
            STR_maneuver(index).process_noise_accelerated_velocity = 8;          
            STR_maneuver(index).process_noise = @(process_noise_angular,process_noise_accelerated_velocity)...
                [[normrnd(0,((angular_velocity/180*pi*T-sin(angular_velocity/180*pi*T))/(angular_velocity/180*pi)^3 * process_noise_angular));...
                normrnd(0,((1-cos(angular_velocity/180*pi*T))/(angular_velocity/180*pi)^2* process_noise_angular));...
                normrnd(0,(sin(angular_velocity/180*pi*T)/(angular_velocity/180*pi) * process_noise_angular))];
                [normrnd(0,((angular_velocity/180*pi*T-sin(angular_velocity/180*pi*T))/(angular_velocity/180*pi)^3 * process_noise_angular));...
                normrnd(0,((1-cos(angular_velocity/180*pi*T))/(angular_velocity/180*pi)^2* process_noise_angular));...
                normrnd(0,(sin(angular_velocity/180*pi*T)/(angular_velocity/180*pi) * process_noise_angular))];
                [normrnd(0,(0.5*T^2*process_noise_accelerated_velocity));
                0;
                0]];
            process_noise = STR_maneuver(index).process_noise(STR_maneuver(index).process_noise_angular,STR_maneuver(index).process_noise_accelerated_velocity); 
            xlog{t} =STR_maneuver(index).F*xlog{t-1} + 1 * process_noise ;
         end
         
         x = xlog{t};
         if abs(x(2)) >5 * 340 ||  abs(x(5)) >5 * 340 || abs(x(8)) >5 * 340
             if abs(x(2)) >5 * 340
                x_lastTime = xlog{t-1};
                x_lastTime(3) = -x_lastTime(3);
                xlog{t-1} = x_lastTime;
             end
             if abs(x(5)) >5 * 340
                x_lastTime = xlog{t-1};
                x_lastTime(6) = -x_lastTime(6);
                xlog{t-1} = x_lastTime;
             end
             if abs(x(8)) >5 * 340
                x_lastTime = xlog{t-1};
                x_lastTime(9) = -x_lastTime(9);
                xlog{t-1} = x_lastTime;
             end
            F_CA = [1, T, 0.5*T^2;...
                    0, 1, T;...
                    0, 0, 1];                     
            F = [F_CA,zeros(3,6);
                zeros(3,3),F_CA,zeros(3,3);
                zeros(3,6),F_CA];                 
            xlog{t} = F * xlog{t-1} + 1 * process_noise; 
            x = xlog{t};
            if (abs(x(2)) >5 * 340 ||  abs(x(5)) >5 * 340 || abs(x(8)) >5 * 340)
                disp("wrong");
            end
         end
         x = xlog{t};
         if abs(x(3)) >5 * 9.8 ||  abs(x(6)) >5 * 9.8 || abs(x(9)) >5 * 9.8             
             if abs(x(3)) >5 * 9.8
                x = xlog{t};
                x(3) = sign(x(3)) * 5 * 9.8;
                xlog{t} = x;
             end
             if abs(x(6)) >5 * 9.8
                x = xlog{t};
                x(6) = sign(x(6)) * 5 * 9.8;
                xlog{t} = x;
             end
             if abs(x(9)) >5 * 9.8
                x = xlog{t};
                x(9) = sign(x(9)) * 5 * 9.8;
                xlog{t} = x;
             end   
             x = xlog{t};
            if (abs(x(3)) >5 * 9.8 ||  abs(x(6)) >5 * 9.8 || abs(x(9)) >5 * 9.8)  
                disp("wrong");
            end
         end         
         x = xlog{t};
        [measlog{t}] = H*xlog{t};    
        result = [sqrt(x(1,1)^2+x(4,1)^2+x(7,1)^2);...
        (atan(x(1,1)/x(4,1)));...
        (atan(x(7,1)/sqrt(x(1,1)^2 + x(4,1)^2)))];
        [measlog_polar{t}] = result;
    end
end

%添加观测噪声
numtruth = 1;
for (t = 1:Ttotal)
    temp=measlog{t};
    for num =1:numtruth
        while 1
            result = [sqrt(temp(1,num)^2+temp(2,num)^2+temp(3,num)^2);...
                    (atan(temp(1,num)/temp(2,num))*180/pi);...
                    (atan(temp(3,num)/sqrt(temp(1,num)^2 + temp(2,num)^2))*180/pi)];
            r = result(1)+normrnd(0,W(1));theta = result(2)+normrnd(0,W(2));phi = result(3)+normrnd(0,W(3));
            if (temp(1,num) >=0 && temp(2,num)>=0)
                theta = theta;
            elseif(temp(1,num) <0 && temp(2,num)>=0)
                theta = theta;
            elseif(temp(1,num) <0 && temp(2,num)<0)
                theta = -180 + theta;
            else
                theta = 180 + theta;
            end   
            temp1=[r*sin(theta*pi/180)*cos(phi*pi/180);r*cos(theta*pi/180)*cos(phi*pi/180);r*sin(phi/180*pi)];
            if ~isnan(temp1)
                temp(:,num) = temp1;
                break;
            end
        end
    end
    [measlog_polar{t}] = [r ;theta; phi];
    measlog{t} = temp;
end

%check
for (t = 1:Ttotal)
    temp = xlog{t};
    temp1 = measlog{t};
    temp2 = measlog_polar{t};
    if numel(find(isnan(temp)))~=0   
       disp('wrong') 
    end
    if numel(find(isnan(temp1)))~=0      
       disp('wrong') 
    end   
    if numel(find(isnan(temp2)))~=0      
       disp('wrong') 
    end        
end

%     绘图
z=[];
x=[];
for i=1:Ttotal
    temp1=xlog{i};
    temp=measlog{i};
    for j=1:numtruth
        z(:,i,j)=temp(:,j);
        x(:,i,j)=temp1(:,j);
    end   
end    
for i=1:numtruth
   p(i)=plot3(x(1,:,i),x(4,:,i),x(7,:,i),'b-','LineWidth',1);
   hold on;
end
grid on;
legend([p(1)],'target4');  
xlabel('m');ylabel('m');zlabel('m');


%产生场景
savedata = [scene * ones(Ttotal,1) (cell2mat(measlog_polar))' (cell2mat(xlog))'];
savepath= ['E:\个人\XD\作业\目标跟踪\Dataset\scene' num2str(scene,'%03d') '.mat'];
save(savepath,'savedata')
