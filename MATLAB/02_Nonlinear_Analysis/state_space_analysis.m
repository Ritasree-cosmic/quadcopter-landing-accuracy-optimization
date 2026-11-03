%% state_space_analysis.m
% Stability and Controllability Analysis
% of the Linearized Quadcopter at Hover

clear;
clc;
close all;

%% Load linearized model

load('linearized_quadcopter.mat');

%% Display A and B matrices

fprintf('\n========================================\n');
fprintf('     LINEARIZED QUADCOPTER ANALYSIS\n');
fprintf('========================================\n');

fprintf('\nA matrix:\n');
disp(A_hover);

fprintf('\nB matrix:\n');
disp(B_hover);

%% ---------------------------------------------------------
% 1. Stability Analysis
% ----------------------------------------------------------

eigenvalues = eig(A_hover);

fprintf('\nEigenvalues of A:\n');
disp(eigenvalues);

%% Count eigenvalues

positive_eigenvalues = sum(real(eigenvalues) > 1e-8);
negative_eigenvalues = sum(real(eigenvalues) < -1e-8);
zero_eigenvalues = sum(abs(real(eigenvalues)) <= 1e-8);

fprintf('Positive real-part eigenvalues = %d\n', ...
    positive_eigenvalues);

fprintf('Negative real-part eigenvalues = %d\n', ...
    negative_eigenvalues);

fprintf('Zero real-part eigenvalues     = %d\n', ...
    zero_eigenvalues);

%% Stability conclusion

if positive_eigenvalues > 0
    fprintf('\nStability conclusion:\n');
    fprintf('The linearized system is UNSTABLE.\n');

elseif negative_eigenvalues == length(eigenvalues)
    fprintf('\nStability conclusion:\n');
    fprintf('The linearized system is ASYMPTOTICALLY STABLE.\n');

else
    fprintf('\nStability conclusion:\n');
    fprintf(['The system has eigenvalues on the imaginary axis.\n' ...
             'The hover linearization is NOT asymptotically stable.\n']);
end

%% ---------------------------------------------------------
% 2. Controllability Analysis
% ----------------------------------------------------------

Co = ctrb(A_hover, B_hover);

rank_Co = rank(Co);

n = size(A_hover, 1);

fprintf('\n========================================\n');
fprintf('       CONTROLLABILITY ANALYSIS\n');
fprintf('========================================\n');

fprintf('\nControllability matrix rank = %d\n', rank_Co);
fprintf('Number of states             = %d\n', n);

if rank_Co == n
    fprintf('\nConclusion:\n');
    fprintf('The linearized quadcopter is FULLY CONTROLLABLE.\n');
else
    fprintf('\nConclusion:\n');
    fprintf('The linearized quadcopter is NOT fully controllable.\n');
end

%% ---------------------------------------------------------
% 3. Controllability Matrix
% ----------------------------------------------------------

fprintf('\nControllability matrix:\n');
disp(Co);

%% ---------------------------------------------------------
% 4. Controllability Gramian
% ----------------------------------------------------------

fprintf('\n========================================\n');
fprintf('       CONTROLLABILITY GRAMIAN\n');
fprintf('========================================\n');

try

    Wc = gram(ss(A_hover, B_hover, ...
        eye(12), zeros(12,4)), 'c');

    fprintf('\nControllability Gramian:\n');
    disp(Wc);

catch

    fprintf('\nControllability Gramian could not be calculated\n');
    fprintf('because the system contains marginal/unstable modes.\n');

end

fprintf('\n========================================\n');
fprintf('             ANALYSIS COMPLETE\n');
fprintf('========================================\n');