function [der] = drv(log_mat,enum,step)
%UNTİTLED Summary of this function goes here
%   Detailed explanation goes here
size = length(log_mat(:,enum));
der = zeros(round(size/step),1);
j=1;
for i=1+step:step:size
    diff = log_mat(i,enum) - log_mat(i-step,enum);
    dt   = log_mat(i,Telem.time_millis)   - log_mat(i-step,Telem.time_millis);
    diff = 1000*diff/dt;
    der(j) = diff;
    j = j+1;
end

plot(der);
end

