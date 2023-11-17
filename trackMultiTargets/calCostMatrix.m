function [costMatrix,flagMatrix] = calCostMatrix(tarTracks,clusterSet, measurentSet, obIndexSet,H,Chi_large)
%计算代价矩阵 即欧氏距离矩阵
%矩阵沿着列方向为航迹 行方向为观测点迹
costMatrix = zeros(length(clusterSet),length(obIndexSet));
flagMatrix = zeros(length(clusterSet),length(obIndexSet));
for i =1 : length(clusterSet)
    for j = 1 : length(obIndexSet)
        [D,flag] = Related_gate_track2ob(tarTracks,clusterSet(i),measurentSet,obIndexSet(j),H,Chi_large);
        costMatrix(i,j) = D;
        flagMatrix(i,j) = flag;
    end
end
end

