function d_out = DCont(error)
d_out = zeros(1,length(error));
N = 1000;
Kd = 0.02;
st = 0.05;
de_int = 0;
for n=2:length(error)
    e_roll = error(n);
    e_roll_ = error(n-1);
    d_out(n) = N * (Kd * e_roll - de_int);
   % disp(d_out(n));
    d_out(n) = e_roll;% - e_roll_;
    d_out(n) = Kd * d_out(n);
   % disp(d_out(n));
    de_int = de_int + d_out(n)*st;
end

end

