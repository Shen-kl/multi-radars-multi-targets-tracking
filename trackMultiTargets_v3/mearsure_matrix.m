function H = mearsure_matrix(x,y,z)
% 量测矩阵
% H = [sqrt(x^2+y^2+z^2);atan(x/y);atan(z/sqrt(x^2+y^2))];
R = sqrt(x^2 + y^2 + z^2);
A =atan(x/y);
if x>=0  && y>=0
    A = A;
elseif x<0 && y>=0
    A = A;
elseif x<0 && y<0
    A = -pi + A;
else
    A= pi + A;
end
E = atan(z/sqrt(x^2 + y^2));
H = [R; A; E];
end