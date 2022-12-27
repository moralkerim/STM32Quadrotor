function [yaw_lift] = lift_yaw(enum,log_mat)
last = 0;
size = length(log_mat(:,enum));
yaw_lift = log_mat(:,enum);
movavgWindow_overlap = dsp.MovingAverage(5,4);
    for i=2:size    
        angle = log_mat(i,enum);
        avg = movavgWindow_overlap(angle);
        while(angle < avg-rad2deg(pi))
            angle = angle + 2*rad2deg(pi);
        end
        while(angle > avg+rad2deg(pi))
            angle = angle - 2*rad2deg(pi);
        end
        %last = angle;
        yaw_lift(i) = angle;
    end
plot(yaw_lift); hold on;
end

