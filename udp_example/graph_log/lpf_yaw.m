function [out] = lpf_yaw(arr)
%UNTİTLED2 Summary of this function goes here
%   Detailed explanation goes here
out = zeros(1,length(arr));
for n=2:length(arr)
    x = arr(n);
    x_ = arr(n-1);
    y_ = out(n-1);
    %y = (pi*x)/(pi + 80) + (160*pi*(x/(pi - 80) - (-(pi - 80)/(pi + 80))^n/(pi - 80)))/(pi + 80);
    %y = 800*f_lpf*pi*(x/(pi*f_lpf - 400) - (-(pi*f_lpf - 400)/(pi*f_lpf + 400))^n/(pi*f_lpf - 400))/(pi*f_lpf + 400) + (f_lpf*pi*x)/(pi*f_lpf + 400);
    %y = (2347303*(1457/2000)^n)/7285000 - (1358*x)/7285;
    %y = (2024901*(4361/10000)^n)/2180500 - (2820*x)/4361;  %50Hz
    %y = (36388629*(5219/10000)^n)/52190000 - (2391*x)/5219; %40 Hz
    y = 0.8544*y_ + 0.07282 * x + 0.07282 * x_;
    out(n) = y;
end

end

