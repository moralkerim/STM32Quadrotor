function [der] = drv(log_mat,enum)
%UNTİTLED Summary of this function goes here
%   Detailed explanation goes here
size = length(log_mat(:,enum));
der = zeros(round(size/10),1);
j=1;
for i=11:10:size
    diff = log_mat(i,enum) - log_mat(i-10,enum);
    dt   = log_mat(i,35)   - log_mat(i-10,35);
    diff = 1000*diff/dt;
    der(j) = diff;
    j = j+1;
end

plot(der);
end

