x = 1:0.01:25;
y = sin(x);
n = numel(x);
figure;
for i = 1:n
    plot(x(1:i),y(1:i),'-r');
    xlim([0 25]);
    ylim([-1.1 1.1]);
    drawnow;
end