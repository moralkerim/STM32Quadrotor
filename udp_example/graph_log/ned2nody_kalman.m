syms DCM11 DCM12 DCM13 DCM21 DCM22 DCM23 DCM31 DCM32 DCM33
DCM = [DCM11, DCM12, DCM13;
       DCM21, DCM22, DCM23;
       DCM31, DCM32, DCM33];
assume(DCM,'real');

syms x y z
xned = [x,y,z]';
assume(xned,'real');

xbody = DCM'*xned