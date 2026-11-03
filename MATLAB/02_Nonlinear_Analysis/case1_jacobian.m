clc;
clear;
close all;

syms x1 x2 mu

f = [x2;
     mu*(1-x1^2)*x2-x1];

x = [x1 x2];

A = jacobian(f,x);

disp('Jacobian Matrix')
pretty(A)