clc;
ln = length(log_mat(:,1));
zerov = zeros(ln,1);
yaw = deg2rad(log_mat(:,Telem.yaw));
pitch = deg2rad(-log_mat(:,Telem.pitch));
roll  = deg2rad(log_mat(:,Telem.roll));
euler = [yaw,pitch,roll];
quat = eul2quat(euler);
viewer = HelperOrientationViewer;

for i=1:ln
    Q = quaternion(quat(i,:));
    viewer(Q);
%     figure(1);
%     hold on
%     plot(i,rad2deg(yaw(i)),'*r');
%     xlim([0 ln])
%     ylim([-40 40])
%     drawnow;
%     disp(rad2deg(yaw(i)));
    pause(.01);

end