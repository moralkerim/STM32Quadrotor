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
DCM = DCM';
%[roll pitch yaw] = dcm2angle(DCM)
DCM = simplify(DCM,'Steps',500);



DMC11 = DCM(1,1)
DMC12 = DCM(1,2)
DMC13 = DCM(1,3)
DMC21 = DCM(2,1)
DMC22 = DCM(2,2)
DMC23 = DCM(2,3)
DMC31 = DCM(3,1)
DMC32 = DCM(3,2)
DMC33 = DCM(3,3)

%  threeaxisrot( DCM(1,2,:), DCM(1,1,:), -DCM(1,3,:), ...
%                            DCM(2,3,:), DCM(3,3,:), ...
%                           -DCM(2,1,:), DCM(2,2,:));
%                       
% function  threeaxisrot(r11, r12, r21, r31, r32, r11a, r12a)
% disp(r11)
% %     % find angles for rotations about X, Y, and Z axes
% %     r1 = atan2( r11, r12 );
% %     r2 = asin( r21 );
% %     r3 = atan2( r31, r32 );
% end