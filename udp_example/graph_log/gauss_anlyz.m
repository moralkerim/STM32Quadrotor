function [M,V] = gauss_anlyz(enum,log_mat)
    sum1=0;
    for i=1:length(log_mat(:,enum))
      sum1=sum1+log_mat(i,enum);
    end
    M=sum1/length(log_mat(:,enum)); %the mean
    sum2=0;
    for i=1:length(log_mat(:,enum))
        sum2=sum2+ (log_mat(i,enum)-M)^2;
    end
    V=sum2/length(log_mat(:,enum)); %Varaince
end

