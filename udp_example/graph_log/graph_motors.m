function [] = graph_motors(log_mat)
    figure();
    graph_telem(Telem.w1,'s',log_mat);
    graph_telem(Telem.w2,'s',log_mat);
    graph_telem(Telem.w3,'s',log_mat);
    graph_telem(Telem.w4,'s',log_mat);
    title('Motor pwm');
    legend('w1','w2','w3','w4');

end