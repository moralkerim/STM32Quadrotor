syms yaw pitch roll real;
cp = cos(pitch); sp = sin(pitch);
cr = cos(roll);  sr = sin(roll);

Rz = eye(3);

Ry = [cp 0 -sp;
      0  1   0;
      sp 0  cp];

Rx = [1  0  0;
      0  cr sr;
      0 -sr cr];

Rzyx = Rz*Ry*Rx