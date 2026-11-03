function [Fx,Fy,Fz] = wind_disturbance(time)

% ============================================================
% EXTERNAL DISTURBANCE / WIND MODEL
%
% Represents an equivalent external force acting on the
% quadcopter during landing.
%
% Output:
% Fx - disturbance force in X direction (N)
% Fy - disturbance force in Y direction (N)
% Fz - disturbance force in Z direction (N)
% ============================================================

%% Horizontal wind disturbance

Fx = 0.08*sin(0.8*time);

Fy = 0.06*cos(0.6*time);

%% Vertical disturbance

Fz = 0;

end