classdef Hungry
    %UNTITLED 此处显示有关此类的摘要
    %   此处显示详细说明
    properties
        C
        row_uncovered
        col_uncovered
        Z0_r
        Z0_c
        path
        marked
    end
    
    methods
        function obj = Hungry(cost_matrix)
            obj.C = cost_matrix;
            n = size(obj.C,1);
            m = size(obj.C,2);
            obj.row_uncovered = ones(n,1);
            obj.col_uncovered = ones(m,1);
            obj.Z0_r = 0;
            obj.Z0_c = 0;
            obj.path = zeros(n + m, 2);
            obj.marked = zeros(n, m);
        end
        
        function obj = clear_covers(obj)
            %Clear all covered matrix cells"""
            obj.row_uncovered(:) = true;
            obj.col_uncovered(:) = true;
        end
    end
end

