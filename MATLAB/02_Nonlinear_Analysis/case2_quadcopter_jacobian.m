clc;
clear;
close all;

syms phi p a b

f = [p;
    -a*sin(phi)-b*p];

x = [phi p];

A = jacobian(f,x);

disp('Symbolic Jacobian Matrix')

pretty(A)

%% Visualization

figure

axis off

text(0.1,0.8,...
'Jacobian Matrix A =')

text(0.1,0.5,...
'| 0      1 |')

text(0.1,0.3,...
'| -a cos(phi)   -b |')

title('Jacobian Linearization')