d = log_mat(:,Telem.of_range)/1000;
xp = log_mat(:,Telem.of_motion_y);
a = deg2rad(42);
W = 30;
K=1;
wm = rad2deg(K*2/W*tan(a/2).*xp);

%75 ms delay