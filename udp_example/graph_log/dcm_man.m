accX = log_mat(:,Telem.accX);
accY = log_mat(:,Telem.accY);
accZ = log_mat(:,Telem.accZ);

magX = log_mat(:,Telem.magX);
magY = log_mat(:,Telem.magY);
magZ = log_mat(:,Telem.magZ);

ln = length(log_mat(:,1));
yaw_dcm = zeros(1,ln);
for i=1:ln
    acctop = sqrt(accX(i)*accX(i)+accY(i)*accY(i)+accZ(i)*accZ(i));
    DCM11 = (magX(i)*accY(i)*accY(i) - accX(i)*magY(i)*accY(i) + magX(i)*accZ(i)*accZ(i) - accX(i)*magZ(i)*accZ(i))/(acctop*sqrt(pow2(accX(i)*magY(i) - accY(i)*magX(i)) + pow2(accX(i)*magZ(i) - accZ(i)*magX(i)) + pow2(accY(i)*magZ(i) - accZ(i)*magY(i))));
    DCM21 = -(accY(i)*magZ(i) - accZ(i)*magY(i))/sqrt(pow2(accX(i)*magY(i) - accY(i)*magX(i)) + pow2(accX(i)*magZ(i) - accZ(i)*magX(i)) + pow2(accY(i)*magZ(i) - accZ(i)*magY(i)));
    yaw_dcm(i) = rad2deg(atan2(DCM21,DCM11));
end
function y = pow2(x)
    y = x*x;
end