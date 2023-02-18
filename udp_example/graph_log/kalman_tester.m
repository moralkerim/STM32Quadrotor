%close all;
%1 is log_mat, 0 is ardupilot log.
LOG_TYPE =1;
[roll_m,roll_bias_m,rate_m,gyroX,roll_acc,S11_m,S12_m,S21_m,S22_m,S13_m,S23_m,S31_m,S32_m,S33_m] = deal(0);
size = length(log_mat(:,Telem.roll_gyro));
kalman_roll = zeros(size,1);
roll_acc_v = zeros(size,1);
roll_comp = zeros(size,1);
kalman_rate = zeros(size,1);
kalman_bias  = zeros(size,1);
Icov_k = zeros(2,2,size);
v_k = zeros(2,1,size);
COMP_PAR = 0.998;
GYRO_BIAS = 0.15;
ACC_BIAS = 0.45;
roll_gyro = 0;
kalman_acc_thres = 0.3;
g = 9.81;
update_enable = 0;
dt_mean = 0;

for i=2:size
    if(LOG_TYPE == 0)
        t = IMU(i,2);
        t_ = IMU(i-1,2);
        accX = IMU(i,6);
        accY = IMU(i,7);
        accZ = IMU(i,8);
        gyroX = (IMU(i,3)) - deg2rad(GYRO_BIAS);

    else
        t = log_mat(i,Telem.time_millis);
        t_ = log_mat(i-1,Telem.time_millis);
        accX = log_mat(i,Telem.accX);
        accY = log_mat(i,Telem.accY);
        accZ = log_mat(i,Telem.accZ);
        gyroX = deg2rad(log_mat(i,Telem.roll_gyro));
    end
    dt = (t - t_)/1000;
    dt_mean =  dt_mean + dt;

   % dt = 20/1000;
    acctop = sqrt(accX^2 + accY^2 + accZ^2);

    %roll_acc = rad2deg(atan(accY/accZ));
    roll_acc = (asin(accY/g));
    if(acctop < (1+kalman_acc_thres) * g && acctop > (1-kalman_acc_thres) * g)
        update_enable = 1;
    else
        update_enable = 0;
    end
    
    %if(i>5000)
        %if(imag(roll_acc) == 0)
            roll_acc_v(i) = roll_acc;
            roll_eski = roll_comp(i-1);
            roll_comp(i)=(roll_gyro+roll_eski)*COMP_PAR+roll_acc*(1-COMP_PAR);
            [roll_m,roll_bias_m,S11_m,S12_m,S21_m,S22_m,v,Icov] = ...
            Kalman_Filter_disc(i,roll_m,roll_bias_m,gyroX,roll_acc,S11_m,S12_m,S21_m,S22_m,dt,update_enable);
            kalman_roll(i) = roll_m;
            %kalman_rate(i) = rate_m;
            kalman_bias(i) = roll_bias_m;
            roll_gyro = gyroX*dt;
       % else
       %     roll_comp(i) = roll_comp(i-1);
       % end
    %else
       % roll_comp(i) = roll_acc;
       % kalman_roll(i) = roll_acc;
   % end
    
%     Icov_k(:,:,i) = Icov;
%     v_k(:,:,i) = v;


    
    
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
figure;
if(LOG_TYPE == 0)
    plot(ATT(:,3)); hold on
else
    plot(log_mat(:,Telem.bno_roll)); hold on;
end
plot(rad2deg(kalman_roll)); hold on
legend('ATT','kalman');
fprintf("dt: %.4f\n", dt_mean/(size-1));
figure;
%plot(bias_mm300); hold on
plot(rad2deg(kalman_bias)); hold on
plot(bias_log); hold on;
legend('ATT','kalman', 'log');
%yyaxis right;
%plot(rad2deg(roll_comp));

% scov = zeros(2,1,size);
% for i=1:size
%     scov(:,:,i) = Icov_k(:,:,i)^-0.5*v_k(:,:,i);
%     %scov1(i,1) = scov(1,1,i);
% end
% scov1(:,1) = scov(1,1,:);
% scov2(:,1) = scov(2,1,:);
% 
% t_sig_a = sa*ones(size);
% plot(scov1); hold on;
% plot(t_sig_a,'b--'); plot(-t_sig_a,'b--');
% title('Angle');
% figure(2);
% 
% t_sig_r = sr*ones(size);
% plot(scov2); hold on;
% plot(t_sig_r,'b--'); plot(-t_sig_r,'b--');
% title('Rate');
% %Scaled cov calculation
% 
% %% 
% close all;
% plot(kalman_rate); hold on;
% plot(roll_gyro)
% legend('kalman','gyro');
% 
% figure(2);
% plot(kalman_roll); hold on;
% plot(roll)
% plot(roll_comp)
% legend('kalman','uav','comp');
% % 
% 
% %%
% close all;
% plot(kalman_bias); hold on;
% plot(roll_bias);
% legend('kalman','uav');
% %
% %%
% close all;
% plot(kalman_rate); hold on;
% plot(roll_rate)
% legend('kalman','uav');
% 
% figure(4);
% plot(kalman_roll); hold on;
% plot(roll_acc)
% legend('kalman','acc');
% %