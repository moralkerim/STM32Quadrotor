yaw_euler = zeros(1,length(log_mat(:,1)));
pitch_euler = zeros(1,length(log_mat(:,1)));
roll_euler  = zeros(1,length(log_mat(:,1)));
accxm = log_mat(:,Telem.accX);
accym = log_mat(:,Telem.accY);
acczm = log_mat(:,Telem.accZ);
magxm = log_mat(:,Telem.magX);
magym = log_mat(:,Telem.magY);
magzm = log_mat(:,Telem.magZ);
for i=1:length(log_mat(:,1))
        %     [          cy*cz,          cy*sz,            -sy]
        %     [ sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx]
        %     [ sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx]
try
    DCM_N = double(subs(DCM,[accX accY accZ magX magY magZ], [accxm(i) accym(i) acczm(i) magxm(i) magym(i) magzm(i)]));
catch E
    disp(i);
    rethrow(E);
   
end
ROT = DCM_N';
[yaw_euler(i),pitch_euler(i),roll_euler(i)] = dcm2angle(ROT);
%x = (DCM_N(3,2)^2+DCM_N(3,3)^2)^(1/2);
%pitch_euler2(i) = atan2(DCM_N(3,1),x);
%pitch_euler2(i) = asin(-DCM_N(3,1));
end
