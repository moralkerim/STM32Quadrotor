clear all; clc
%clc

%Example LOG
log_str = '01.12.2022.16.13.48.txt';

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

log_mat = log_mat(2:end,:);

%Clean Log
[log_mat] = clean_log(Telem.roll,log_mat,1001);
[log_mat] = clean_log(Telem.pitch,log_mat,1000);
[log_mat] = clean_log(Telem.pitch_comp,log_mat,1000);
[log_mat] = clean_log(Telem.roll_acc,log_mat,1000);
[log_mat] = clean_log(Telem.roll_gyro,log_mat,1000);
[log_mat] = clean_log(Telem.roll_rate,log_mat,1000);
[log_mat] = clean_log(Telem.roll_comp,log_mat,1000);
[log_mat] = clean_log(Telem.time_millis,log_mat,100000);

%Innovations
roll_rate = log_mat(:,Telem.roll_rate);
roll_gyro = log_mat(:,Telem.roll_gyro);
roll_comp = log_mat(:,Telem.roll_comp);

pitch_rate = log_mat(:,Telem.pitch_rate);
pitch_gyro = log_mat(:,Telem.pitch_gyro);
pitch_comp = log_mat(:,Telem.pitch_comp);
pitch = log_mat(:,Telem.pitch);

roll = log_mat(:,Telem.roll);
roll_acc = log_mat(:,Telem.roll_acc);
roll_bias = log_mat(:,Telem.sonar_alt);
v_roll = roll_acc-roll;