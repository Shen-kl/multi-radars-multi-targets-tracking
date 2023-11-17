% Author: Shen Kailun
% Date  : 2023-11-15
function [locSet] = Sudoku(locNum, rangeStep, aziStep, distanceMax, distanceMin, aziMax, aziMin)
%函数功能： 以目标航迹所在分区为中心 查找其周围紧邻的分区的编号 分区编号从0开始
%参数说明：  locNum  目标航迹坐在的分区编号 
%           rangeStep 距离维的分区步长
%           aziStep 方向角维的分区步长
    number = fix((distanceMax-distanceMin)/rangeStep);
    locNumMax = fix((aziMax -  aziMin) / aziStep) * number ;
    locSet = [locNum];
    
    if mod(locNum, number) == 0
        if mod(locNum, number) == number - 1 
            if locNum + number > locNumMax
                locSet = locSet;
            else
                locSet = [locSet (locNum + number)];
            end

            if locNum - number < 0
                locSet = locSet;
            else
                locSet = [locSet (locNum - number) ];
            end            
        else
            locSet = [locSet (locNum + 1)];
            if locNum + number > locNumMax
                locSet = locSet;
            else
                locSet = [locSet (locNum + number) ];
                locSet = [locSet (locNum + number + 1) ];
            end

            if locNum - number < 0
                locSet = locSet;
            else
                locSet = [locSet (locNum - number) ];
                locSet = [locSet (locNum - number + 1)];
            end                
        end
    else
        if mod(locNum, number) == number - 1
            locSet = [locSet (locNum - 1)];
            if locNum + number > locNumMax
                locSet = locSet;
            else
                locSet = [locSet (locNum + number) ];
                locSet = [locSet (locNum + number -1)];
            end

            if locNum - number < 0
                locSet = locSet;
            else
                locSet = [locSet (locNum - number) ];
                locSet = [locSet (locNum - number - 1) ];
            end        
        else
            locSet = [locSet (locNum - 1)];
            locSet = [locSet (locNum + 1)];
            if locNum + number > locNumMax
                locSet = locSet;
            else
                locSet = [locSet (locNum + number + 1)];
                locSet = [locSet (locNum + number) ];
                locSet = [locSet (locNum + number -1)];
            end

            if locNum - number < 0
                locSet = locSet;
            else
                locSet = [locSet (locNum - number + 1)];
                locSet = [locSet (locNum - number) ];
                locSet = [locSet (locNum - number - 1) ];
            end               
        end
    end
    locSet = locSet + 1;
end

