function R_c = error_conversion(R_p,Polar_X)
% inpu:R_p:极坐标下的量测噪声
% output:R_c:直角坐标下的量测噪声
% 转换雅可比矩阵
rou = Polar_X(1);
theta = Polar_X(2);
fai = Polar_X(3);
J = [cos(fai)*cos(theta),-rou*cos(fai)*sin(theta),-rou*cos(fai)*sin(theta);...
    cos(fai)*sin(theta),rou*cos(fai)*cos(theta),-rou*sin(fai)*sin(theta);...
    sin(fai),0,rou*cos(fai)];
R_c = J * R_p * J';
end