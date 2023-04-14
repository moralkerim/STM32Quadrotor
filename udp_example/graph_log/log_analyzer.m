%clear all; clc
%clc; close all;

%18.01.2023.14.05.47.txt düşüş logu
%19.01.2023.13.52.24.txt zeytinlik
zeytinlik = '19.01.2023.13.52.24.txt';
swarm = 'swarm3.txt';
swarm_coil = 'swarm_coil.txt';
fine_log = '12.03.2023.17.49.47.txt';
log_str = 'of_calib.txt';
%log_str = zeytinlik;

format='(%f';
end_format = ')';
inc_format = ',%f';
file_read = 0;

while(~file_read)
    try
        file_read = 0;
        log = fopen(log_str,'rt');

        A = textscan(log, append(format,end_format));
        format = append(format,inc_format);
        fclose(log);
        log_mat = cell2mat(A);
        
    catch
        file_read = 1;
        %format = append(format,inc_format);
    end
end

log_mat = log_mat(5:end,:);

%Clean Log
[log_mat] = clean_log(Telem.w1,log_mat,3000);
[log_mat] = clean_log(Telem.w2,log_mat,3000);
[log_mat] = clean_log(Telem.w3,log_mat,3000);
[log_mat] = clean_log(Telem.w4,log_mat,3000);

[log_mat] = clean_log(Telem.accX,log_mat,500);
[log_mat] = clean_log(Telem.accY,log_mat,500);
[log_mat] = clean_log(Telem.accZ,log_mat,500);
[log_mat] = clean_log(Telem.roll,log_mat,500);
[log_mat] = clean_log(Telem.roll_des,log_mat,500);
[log_mat] = clean_log(Telem.pitch,log_mat,1000);
[log_mat] = clean_log(Telem.pitch_comp,log_mat,1000);
[log_mat] = clean_log(Telem.roll_acc,log_mat,1000);
[log_mat] = clean_log(Telem.roll_gyro,log_mat,1000);
[log_mat] = clean_log(Telem.roll_rate,log_mat,1000);
[log_mat] = clean_log(Telem.yaw_gyro,log_mat,1000);
[log_mat] = clean_log(Telem.yaw_rate,log_mat,1000);
[log_mat] = clean_log(Telem.roll_comp,log_mat,1000);
[log_mat] = clean_log(Telem.sonar_alt,log_mat,1000);
%[log_mat] = clean_log(Telem.time_millis,log_mat,100000);

%Innovations
% roll_rate = log_mat(:,Telem.roll_rate);
% roll_gyro = log_mat(:,Telem.roll_gyro);
% roll_comp = log_mat(:,Telem.roll_comp);
% 
% pitch_rate = log_mat(:,Telem.pitch_rate);
% pitch_gyro = log_mat(:,Telem.pitch_gyro);
% pitch_comp = log_mat(:,Telem.pitch_comp);
% pitch = log_mat(:,Telem.pitch);
% 
% roll = log_mat(:,Telem.roll);
% roll_acc = log_mat(:,Telem.roll_acc);
% roll_bias = log_mat(:,Telem.sonar_alt);
% v_roll = roll_acc-roll;
% e_roll = log_mat(:,Telem.roll_des) - log_mat(:,Telem.roll);
% e_roll_rate = log_mat(:,Telem.roll_des_rate) - log_mat(:,Telem.roll_rate);
% angle_ff_roll = log_mat(:,Telem.vz);
% bno_e_roll = log_mat(:,Telem.roll_des) - log_mat(:,Telem.bno_roll);
roll_omega_rad = pi/180.*log_mat(:,Telem.roll_gyro);
roll_rate_rad = pi/180.*log_mat(:,Telem.roll_rate);
roll_noise_rad = roll_omega_rad - roll_rate_rad;

yaw_omega_rad = pi/180.*log_mat(:,Telem.yaw_gyro);
yaw_rate_rad = pi/180.*log_mat(:,Telem.yaw_rate);
yaw_noise_rad = yaw_omega_rad - yaw_rate_rad;