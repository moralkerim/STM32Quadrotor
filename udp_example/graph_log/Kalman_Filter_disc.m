function [pitch,pitch_bias,rate,S11,S12,S21,S22,S13,S23,S31,S32,S33,v,Icov] = Kalman_Filter_disc(i,pitch_m,pitch_bias_m,rate_m,gyroY,pitch_acc,S11_m,S12_m,S21_m,S22_m,S13_m,S23_m,S31_m,S32_m,S33_m)
sa = 3e-1; 
sb = 5e-2; 
sr = 20;

if (i<150)
   %sb = 1e-5;
    Qa = 1;
    Qg = 1e-2;
else
    Qa = 5e4;
    Qg = 200;
end


dt = 0.0355;

a_m = pitch_m;
b_m  = pitch_bias_m;
r_m = rate_m;
x_m = [a_m b_m r_m]';

S_m = [S11_m S12_m S13_m; S21_m S22_m S23_m; S31_m S32_m S33_m];
R = [sa 0 0; 0 sb 0; 0 0 sr];

Q = [Qa 0; 0 Qg];

z = [pitch_acc gyroY]';

A = [1 0 dt; 0 1 0; 0 0 1];
B = [0 0 0]';
C = [1 0 0; 0 1 1];

u = 0;

x = A * x_m + B*u;
S = A*S_m*A'+ R;

Kt = S*C'*inv(C*S*C'+Q);

Icov = C*S*C'+Q;
v = z - C*x;

x = x + Kt * (z - C*x);
S = (eye(3)-Kt*C)*S;

pitch = x(1);
pitch_bias = x(2);
rate = x(3);

S11 = (S(1,1));
S12 = (S(1,2));
S13 = (S(1,3));
S21 = (S(2,1));
S22 = (S(2,2));
S23 = (S(2,3));
S31 = (S(3,1));
S32 = (S(3,2));
S33 = (S(3,3));