function ind= float2ind(flt,res,f_min)
    % float2ind 浮点坐标转ind
    %   从1开始编号
    ind=ceil((flt-f_min)/res)+1;
end