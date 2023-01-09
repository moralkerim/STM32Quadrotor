function yaw_warped = unwrap_yaw(enum,log_mat)
  % Define a threshold value that determines when the angle has crossed the discontinuity
  threshold = 50;
  size = length(log_mat(:,enum));
  yaw_warped = log_mat(:,enum);
  jump = false;
  jump_count = 0;

  for i=2:size
      yaw = log_mat(i,enum);
      prev_yaw = log_mat(i-1,enum);
      % Check if the yaw angle has crossed the discontinuity
      if(~jump)
          yaw_warped(i) = yaw + jump_count*360;
          if abs(yaw - prev_yaw) > threshold
                jump = true;
                jump_val = prev_yaw;

          end

      else
          if(abs(yaw) > 140)
                jump = false;
                if yaw > jump_val
                  jump_count = jump_count-1;
                else
                  jump_count = jump_count+1;
                end
                yaw_warped(i) = yaw + jump_count*360;

          else
                yaw_warped(i) = jump_val;

          end
     
      end


  end
end
