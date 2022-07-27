function [b,S] = acc_calib(aiup,aidown)
%UNTİTLED Summary of this function goes here
%   Detailed explanation goes here
g = 128.2051;
b = (aiup + aidown)/2;
S = (aiup - aidown - 2*g)/(2*g);
end

