clear all; clc
log = fopen('26.06.2022.16.55.14.txt','rt');
%(-0.0393805094063282, -4.482371807098389, 0.0, 1000, 1000, 1000, 1000)
A = textscan(log, '(%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f)');
fclose(log);
log_mat = cell2mat(A);
