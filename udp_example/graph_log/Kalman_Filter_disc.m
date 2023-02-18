function [pitch,pitch_bias,S11,S12,S21,S22,v,Icov] = Kalman_Filter_disc(i,pitch_m,pitch_bias_m,gyroY,pitch_acc,S11_m,S12_m,S21_m,S22_m,dt,update_enable)

sa = 1;  sr=7e-1; sb = 1e-1;

Qa = 1e6; Qg = 1e1;



%dt = 0.0355;

a_m = pitch_m;
b_m  = pitch_bias_m;
%r_m = rate_m;
x_m = [a_m b_m]';

S_m = [S11_m S12_m; S21_m S22_m;];
R = [sa 0; 0 sb];

Q = [Qa];

z = [pitch_acc]';

A = [1 -dt; 0 1];
B = [dt 0]';
C = [1 0];

u = gyroY;

x = A * x_m + B*u;
S = A*S_m*A'+ R;

Kt = S*C'*inv(C*S*C'+Q);

Icov = C*S*C'+Q;
v = z - C*x;

if(update_enable)
    x = x + Kt * (z - C*x);
    S = (eye(2)-Kt*C)*S;
end


pitch = x(1);
pitch_bias = x(2);
%rate = x(3);

S11 = (S(1,1));
S12 = (S(1,2));
%S13 = (S(1,3));
S21 = (S(2,1));
S22 = (S(2,2));
%S23 = (S(2,3));
%S31 = (S(3,1));
%S32 = (S(3,2));
%S33 = (S(3,3));