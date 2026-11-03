clc;
clear;

a = 2;
b = 0.5;

%% Equilibrium at phi = 0

A = [0 1;
    -a -b];

disp('Linearized Matrix')

disp(A)

lambda = eig(A);

disp('Eigenvalues')

disp(lambda)

if all(real(lambda)<0)

    disp('Stable Equilibrium')

elseif any(real(lambda)>0)

    disp('Unstable Equilibrium')

else

    disp('Marginally Stable')

end