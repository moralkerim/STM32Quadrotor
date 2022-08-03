function [integration] = integ(enum,log_mat,t)
%UNTİTLED3 Summary of this function goes here
%   Detailed explanation goes here
size = length(log_mat(:,enum));
integration = zeros(size,1);
for i=2:size
    st = t(i) - t(i-1);
    integration(i) = integration(i-1) + log_mat(i,enum) * st;
end
end

