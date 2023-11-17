% Author: Shen Kailun
% Date  : 2023-11-14
function [zone_number]=attribute_location_number(PolarX, distanceStep, aziStep, distanceMax, distanceMin, aziMin)
    %分配分区编号
    %input  PolarX 极坐标系下的坐标
    %input  rangeStep 距离上的分区步长
    %input  aziStep 方位角上的分区步长
    %output number 分区编号 从0开始
    
    R=PolarX(1);
    azi = PolarX(2); % 180 ~ 180
    number1 = fix((R-distanceMin)/distanceStep);
    number2 = fix((azi - aziMin)/aziStep);
    
    number3 = fix((distanceMax-distanceMin)/distanceStep);
    
    zone_number = number2 * number3 + number1 + 1;
end