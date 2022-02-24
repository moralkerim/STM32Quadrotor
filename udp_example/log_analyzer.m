log = fopen('example.txt','rt');
%(-0.0393805094063282, -4.482371807098389, 0.0, 1000, 1000, 1000, 1000)
A = textscan(log, '(%d,%d,%d,%d,%d,%d,%d)');
fclose(log);
log_mat = cell2mat(A);

ax = axes;
ax.LineStyleOrder = {'-'};
hold on;

for i=1:3
    plot(log_mat(:,i))
    hold on;

end
legend('roll','pitch','yaw');
figure(2);

for i=4:7
    plot(log_mat(:,i),'-')
    hold on;

end

legend('w1','w2','w3','w4');