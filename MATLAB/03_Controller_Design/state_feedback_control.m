clc;
clear;
close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LINEARIZED SYSTEM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

A = [0 1;
    -1 -1];

B = [0;
     1];

C = eye(2);
D = [0;0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CONTROLLABILITY CHECK
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Co = ctrb(A,B);

disp('Controllability Matrix:');
disp(Co);

rank_Co = rank(Co);

disp(['Rank = ', num2str(rank_Co)]);

if rank_Co == size(A,1)
    disp('System is CONTROLLABLE');
else
    disp('System is NOT controllable');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% OPEN LOOP SYSTEM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sys_open = ss(A,B,C,D);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DESIRED POLES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

desired_poles = [-2 -3];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STATE FEEDBACK GAIN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

K = place(A,B,desired_poles);

disp('State Feedback Gain K:');
disp(K);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CLOSED LOOP SYSTEM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Acl = A - B*K;

sys_closed = ss(Acl,B,C,D);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% TIME VECTOR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t = 0:0.01:10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INITIAL CONDITION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x0 = [0.5;
      0.2];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% OPEN LOOP RESPONSE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[y_open,t_open,x_open] = initial(sys_open,x0,t);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CLOSED LOOP RESPONSE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[y_closed,t_closed,x_closed] = initial(sys_closed,x0,t);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PLOT x1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure;

subplot(2,1,1);

plot(t_open,x_open(:,1),'r','LineWidth',2);
hold on;

plot(t_closed,x_closed(:,1),'b--','LineWidth',2);

grid on;

xlabel('Time');
ylabel('x1');

title('State x1: Open Loop vs Closed Loop');

legend('Open Loop','Closed Loop');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PLOT x2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

subplot(2,1,2);

plot(t_open,x_open(:,2),'r','LineWidth',2);
hold on;

plot(t_closed,x_closed(:,2),'b--','LineWidth',2);

grid on;

xlabel('Time');
ylabel('x2');

title('State x2: Open Loop vs Closed Loop');

legend('Open Loop','Closed Loop');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PHASE PORTRAIT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure;

plot(x_open(:,1),x_open(:,2),'r','LineWidth',2);
hold on;

plot(x_closed(:,1),x_closed(:,2),'b--','LineWidth',2);

plot(0,0,'ko','MarkerSize',8,'LineWidth',2);

grid on;

xlabel('x1');
ylabel('x2');

title('Phase Portrait: Open Loop vs Closed Loop');

legend('Open Loop','Closed Loop','Equilibrium');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% EIGENVALUES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('Open Loop Eigenvalues:');
disp(eig(A));

disp('Closed Loop Eigenvalues:');
disp(eig(Acl));