function [sonar_filt] = sonar_filt(sonar)
    arr_size = length(sonar);
    sonar_filt = zeros(arr_size,1);
    sonar_filt(1) = sonar(1);
    for i=2:arr_size
        sonar_filt(i) = sonar(i);
        if(sonar_filt(i) > 5 || sonar_filt(i)<0.5)
            sonar_filt(i) = sonar_filt(i-1);
        end
    end
end

