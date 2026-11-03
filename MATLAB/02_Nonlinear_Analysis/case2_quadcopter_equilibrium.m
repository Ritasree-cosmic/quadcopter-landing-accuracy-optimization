clc;
clear;

disp('QUADCOPTER EQUILIBRIUM POINTS')

for k=-2:2

    phi_eq = k*pi;

    fprintf('\n');
    fprintf('Phi = %.4f rad\n',phi_eq);
    fprintf('Roll Rate = 0\n');

end