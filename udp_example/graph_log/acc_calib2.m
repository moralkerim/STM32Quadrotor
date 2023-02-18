Axmax = 9.33; Axmin = -0.35;
Aymax = 9.90; Aymin =  0.4;
Azmax = 10.1; Azmin =  0.9;

Amax = [Axmax Aymax Azmax]';
Amin = [Axmin Aymin Azmin]';

g = 9.81;

for i=1:3
    S = g/(Amax(i)-Amin(i));
    B = ((Amax(i)+Amin(i)) - g/S)/2;
    fprintf('S%d=%.4f\n',i,S);
    fprintf('B%d=%.4f\n',i,B);
end

