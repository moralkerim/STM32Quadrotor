function [yaw_lift] = lift_yaw(enum,log_mat)
last = 0;
size = length(log_mat(:,enum));
yaw_lift = log_mat(:,enum);
    for i=2:size    
        angle = log_mat(i,enum);
        while(angle < last-rad2deg(pi))
            angle = angle + 2*rad2deg(pi);
        end
        while(angle > last+rad2deg(pi))
            angle = angle - 2*rad2deg(pi);
        end
        last = angle;
        yaw_lift(i) = angle;
    end
end