function log_mat2 = delete_same(log_mat)
log_mat2 = log_mat;
same_arr = [];

for i=2:length(log_mat(:,1))
    if(log_mat(i,Telem.time_millis) == log_mat(i-1,Telem.time_millis))
        same_arr(end+1) = i;

    elseif(abs(log_mat(i,Telem.time_millis) - log_mat(i-1,Telem.time_millis)) > 15)
        same_arr(end+1) = i;
    end

end

inc_array = 1:length(same_arr);
inc_array = inc_array - 1;
same_array2 = same_arr - inc_array;

for i=1:length(same_array2)
    log_mat2(same_array2(i),:) = [];
end
end