clc;
clear;
close all;

A = [0 1;
    -2 -3];

B = [0;
     1];

% Controllability matrix
Co = ctrb(A,B);

disp('Controllability Matrix:')
disp(Co)

% Rank
r = rank(Co);

disp('Rank:')
disp(r)

if r == size(A,1)
    disp('System is controllable')
else
    disp('System is NOT controllable')
end