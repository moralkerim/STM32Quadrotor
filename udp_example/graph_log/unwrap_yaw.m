function yaw_warped = unwrap_yaw(enum,log_mat)
  % Define a threshold value that determines when the angle has crossed the discontinuity
  threshold = 130;
  size = length(log_mat(:,enum));
  yaw_warped = log_mat(:,enum);

  for i=2:size
      yaw = log_mat(i,enum);
      prev_yaw = yaw_warped(i-1);
      % Check if the yaw angle has crossed the discontinuity
      if abs(yaw - prev_yaw) > threshold
        % Add or subtract 2*pi to the yaw angle to bring it back into the range (-pi, pi)
        if yaw > prev_yaw
          yaw = yaw - 2*180;
        else
          yaw = yaw + 2*180;
        end
      end
    yaw_warped(i) = yaw;
  end
end
