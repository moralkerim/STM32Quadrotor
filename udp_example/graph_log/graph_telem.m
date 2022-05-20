function graph_telem(enum,style,log_mat)
%UNTİTLED2 Summary of this function goes here
%   Detailed explanation goes here
if(style == 't')
    plot(log_mat(:,Telem.time_millis)./1000,log_mat(:,enum));

else
    plot(log_mat(:,enum));
end
    hold on
end

