%clear all; clc
log = fopen('10.06.2022.13.57.40.txt','rt');
%(-0.0393805094063282, -4.482371807098389, 0.0, 1000, 1000, 1000, 1000)
A = textscan(log, '(%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f)');
fclose(log);
log_mat = cell2mat(A);
