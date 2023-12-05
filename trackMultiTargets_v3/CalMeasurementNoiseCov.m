% Author: Shen Kailun
% Date  : 2023-11-19
function [ mu,measureNoiseCov] = CalMeasurementNoiseCov(R,A,E,rangeNoise,aziNoise,eleNoise)
%函数功能：计算极坐标系转换至直角坐标系下时，观测误差矩阵的转换 
%输入： R 量测径向距离
%       A 量测方位角
%       E 量测俯仰角
%       rangeNoise 距离误差
%       aziNoise 方位角误差 弧度制
%       eleNosie 俯仰角误差 弧度制
        mu=zeros(3,1);
        mu(1)=R*sin(A*pi/180)*cos(E*pi/180)*(exp(-(aziNoise)^2)*exp(-(eleNoise)^2)-exp(-(eleNoise)^2/2)*exp(-(aziNoise)^2/2));
        mu(2)=R*cos(A*pi/180)*cos(E*pi/180)*(exp(-(aziNoise)^2)*exp(-(eleNoise)^2)-exp(-(eleNoise)^2/2)*exp(-(aziNoise)^2/2));
        mu(3)=R*sin(E*pi/180)*(exp(-(eleNoise)^2)-exp(-(eleNoise)^2/2));
        alpha_y=sin(A*pi/180)^2*sinh((aziNoise)^2)+cos(A*pi/180)^2*cosh((aziNoise)^2);
        alpha_x=sin(A*pi/180)^2*cosh((aziNoise)^2)+cos(A*pi/180)^2*sinh((aziNoise)^2);
        alpha_z=sin(E*pi/180)^2*cosh((eleNoise)^2)+cos(E*pi/180)^2*sinh((eleNoise)^2);
        alpha_xy=sin(E*pi/180)^2*sinh((eleNoise)^2)+cos(E*pi/180)^2*cosh((eleNoise)^2);
        beta_y=sin(A*pi/180)^2*sinh(2*(aziNoise)^2)+cos(A*pi/180)^2*cosh(2*(aziNoise)^2);
        beta_x=sin(A*pi/180)^2*cosh(2*(aziNoise)^2)+cos(A*pi/180)^2*sinh(2*(aziNoise)^2);
        beta_z=sin(E*pi/180)^2*cosh(2*(eleNoise)^2)+cos(E*pi/180)^2*sinh(2*(eleNoise)^2);
        beta_xy=sin(E*pi/180)^2*sinh(2*(eleNoise)^2)+cos(E*pi/180)^2*cosh(2*(eleNoise)^2);

        R_xx=(R^2*(beta_x*beta_xy-alpha_x*alpha_xy)+rangeNoise^2*(2*beta_x*beta_xy-alpha_x*alpha_xy))*exp(-2*(aziNoise)^2)*exp(-2*(eleNoise)^2);
        R_yy=(R^2*(beta_y*beta_xy-alpha_y*alpha_xy)+rangeNoise^2*(2*beta_y*beta_xy-alpha_y*alpha_xy))*exp(-2*(aziNoise)^2)*exp(-2*(eleNoise)^2);
        R_zz=(R^2*(beta_z-alpha_z)+rangeNoise^2*(2*beta_z-alpha_z))*exp(-2*(eleNoise)^2);

        R_xy=(R^2*(beta_xy-alpha_xy*exp((aziNoise)^2))+rangeNoise^2*(2*beta_xy-alpha_xy*exp((aziNoise)^2)))*sin(A*pi/180)*cos(A*pi/180)*exp(-4*(aziNoise)^2)*exp(-2*(eleNoise)^2);
        R_xz=(R^2*(1-exp((eleNoise)^2))+rangeNoise^2*(2-exp((eleNoise)^2)))*sin(A*pi/180)*sin(E*pi/180)*cos(E*pi/180)*exp(-(aziNoise)^2)*exp(-4*(eleNoise)^2);
        R_yz=(R^2*(1-exp((eleNoise)^2))+rangeNoise^2*(2-exp((eleNoise)^2)))*cos(A*pi/180)*sin(E*pi/180)*cos(E*pi/180)*exp(-(aziNoise)^2)*exp(-4*(eleNoise)^2);
        measureNoiseCov=[R_xx R_xy R_xz;R_xy R_yy R_yz ;R_xz R_yz R_zz];
end

