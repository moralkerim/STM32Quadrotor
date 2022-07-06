function [log_mat2] = clean_log(enum,log_mat,err_val)
size = length(log_mat(:,enum));
log_mat2 = log_mat;
for i=1:size
    if(abs(log_mat2(i,enum)) > err_val)
        log_mat2(i,enum) = log_mat2(i-1,enum);
    end
end

end

