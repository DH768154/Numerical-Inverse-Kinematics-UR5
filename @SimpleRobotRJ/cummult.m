function T_cum = cummult(T,varargin)
% Matrix Cumulative Multiplication
[row,col,lay] = size(T);

T_cum = NaN([row,col,lay]);
T_cum(:,:,1) = T(:,:,1);
for i = 2:lay
    T_cum(:,:,i) = T_cum(:,:,i-1)*T(:,:,i);
end
end