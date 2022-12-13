close all;

[pitch_m,pitch_bias_m,rate_m,gyroY,pitch_acc,S11_m,S12_m,S21_m,S22_m,S13_m,S23_m,S31_m,S32_m,S33_m] = deal(0);
size = length(log_mat(:,Telem.roll_rate));
kalman_pitch = zeros(size,1);
kalman_rate = zeros(size,1);
kalman_bias  = zeros(size,1);
Icov_k = zeros(2,2,size);
v_k = zeros(2,1,size);

sa = 6e-3; 
sb = 1e-3; 
sr = 20;

C = [1 0 0; 0 1 1];

for i=2:size
    ti =  log_mat(i,Telem.time_millis);
    ti_ = log_mat(i-1,Telem.time_millis);
    dt = (ti-ti_)/1000
    gyroY = log_mat(i,Telem.roll_gyro);
    pitch_acc = log_mat(i,Telem.roll_acc);
    [pitch_m,pitch_bias_m,rate_m,S11_m,S12_m,S21_m,S22_m,S13_m,S23_m,S31_m,S32_m,S33_m,v,Icov] = ...
    Kalman_Filter_disc(i,pitch_m,pitch_bias_m,rate_m,gyroY,pitch_acc,S11_m,S12_m,S21_m,S22_m,S13_m,S23_m,S31_m,S32_m,S33_m,dt);
    kalman_pitch(i) = pitch_m;
    kalman_rate(i) = rate_m;
    kalman_bias(i) = pitch_bias_m;
    
    Icov_k(:,:,i) = Icov;
    v_k(:,:,i) = v;
    
%     S(1,1,i) = S11_m;
%     S(1,2,i) = S12_m;
%     S(1,3,i) = S13_m;
%     S(2,1,i) = S21_m;
%     S(2,2,i) = S22_m;
%     S(2,3,i) = S23_m;
%     S(3,1,i) = S31_m;
%     S(3,2,i) = S32_m;
%     S(3,3,i) = S33_m;
    
end

scov = zeros(2,1,size);
for i=1:size
    scov(:,:,i) = Icov_k(:,:,i)^-0.5*v_k(:,:,i);
    %scov1(i,1) = scov(1,1,i);
end
scov1(:,1) = scov(1,1,:);
scov2(:,1) = scov(2,1,:);

t_sig_a = sa*ones(size);
plot(scov1); hold on;
plot(t_sig_a,'b--'); plot(-t_sig_a,'b--');
title('Angle');
figure(2);

t_sig_r = sr*ones(size);
plot(scov2); hold on;
plot(t_sig_r,'b--'); plot(-t_sig_r,'b--');
title('Rate');
%Scaled cov calculation

%% 
close all;
plot(kalman_rate); hold on;
plot(roll_gyro)
legend('kalman','gyro');

figure(2);
plot(kalman_pitch); hold on;
plot(roll)
plot(roll_comp)
legend('kalman','uav','comp');
% 

%%
close all;
plot(kalman_bias); hold on;
plot(roll_bias);
legend('kalman','uav');
%
%%
close all;
plot(kalman_rate); hold on;
plot(roll_rate)
legend('kalman','uav');

figure(4);
plot(kalman_pitch); hold on;
plot(roll_acc)
legend('kalman','acc');
%