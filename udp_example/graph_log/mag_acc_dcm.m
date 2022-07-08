clc
syms accX accY accZ;
syms magX magY magZ;

acc = [accX accY accZ];
mag = [magX magY magZ];

assume(acc,'real');
assume(mag,'real');

mag_unit = mag./norm(mag);
acc_unit = acc./norm(acc);

D = -acc_unit;
E = cross(D,mag_unit);
E = E./norm(E);
N = cross(E,D);

DCM = [N',E',D'];
%[roll pitch yaw] = dcm2angle(DCM)
DCM = simplify(DCM,'Steps',1000);

DMC11 = DCM(1,1)
DMC12 = DCM(1,2)
DMC13 = DCM(1,3)
DMC21 = DCM(2,1)
DMC22 = DCM(2,2)
DMC23 = DCM(2,3)
DMC31 = DCM(3,1)
DMC32 = DCM(3,2)
DMC33 = DCM(3,3)