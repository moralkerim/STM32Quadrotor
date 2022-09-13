%clear all; clc
clc
log = fopen('09.08.2022.10.08.01.txt','rt');
%(-0.0393805094063282, -4.482371807098389, 0.0, 1000, 1000, 1000, 1000)
A = textscan(log, '(%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f)');
fclose(log);
log_mat = cell2mat(A);

%(sqrt(square(acc[0]) + square(acc[1]) + square(acc[2]))*sqrt(square(acc[0]*mag[1] - acc[1]*mag[0]) + square(acc[0]*mag[2] - acc[2]*mag[0]) + square(acc[1]*mag[2] - acc[2]*mag[1])));
%((accX^2 + accY^2 + accZ^2)^(1/2)*((accX*magY - accY*magX)^2 + (accX*magZ - accZ*magX)^2 + (accY*magZ - accZ*magY)^2)^(1/2))