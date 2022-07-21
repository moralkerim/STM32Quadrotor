magX = log_mat(:,Telem.accX);
magY = log_mat(:,Telem.accY);
magZ = log_mat(:,Telem.accZ);
mag_data = [magX,magY,magZ];
[a,b,exp] = magcal(mag_data)