function ind= float2ind(flt,res,f_min)
    % float2ind ��������תind
    %   ��1��ʼ���
    ind=ceil((flt-f_min)/res)+1;
end