clc;
clear;
close all;

fprintf('\n');
fprintf('=============================================\n');
fprintf(' LANDING PERFORMANCE COMPARISON\n');
fprintf('=============================================\n');

%% Load results

load('final_landing_off_results.mat');
off_x = x_history;
off_t = t;

load('final_landing_kf_off_results.mat');
kf_x = x_history;
kf_t = t;

%% Common landing criterion

landing_altitude = 0.15;
vertical_speed_limit = 0.30;

%% =========================================================
% RAW SENSOR + LQR
% ==========================================================

off_index = find( ...
    off_x(3,:) <= landing_altitude & ...
    abs(off_x(6,:)) <= vertical_speed_limit, ...
    1,'first');

if isempty(off_index)
    off_index = length(off_t);
end

off_X = off_x(1,off_index);
off_Y = off_x(2,off_index);
off_Z = off_x(3,off_index);

off_error = sqrt( ...
    off_X^2 + off_Y^2);

off_time = off_t(off_index);

%% =========================================================
% KF + LQR
% ==========================================================

kf_index = find( ...
    kf_x(3,:) <= landing_altitude & ...
    abs(kf_x(6,:)) <= vertical_speed_limit, ...
    1,'first');

if isempty(kf_index)
    kf_index = length(kf_t);
end

kf_X = kf_x(1,kf_index);
kf_Y = kf_x(2,kf_index);
kf_Z = kf_x(3,kf_index);

kf_error = sqrt( ...
    kf_X^2 + kf_Y^2);

kf_time = kf_t(kf_index);

%% =========================================================
% IMPROVEMENT
% ==========================================================

improvement = ...
    ((off_error-kf_error)/off_error)*100;

%% =========================================================
% DISPLAY
% ==========================================================

fprintf('\nCommon touchdown criterion:\n');
fprintf('Altitude <= %.2f m\n',landing_altitude);
fprintf('|Vertical velocity| <= %.2f m/s\n',vertical_speed_limit);

fprintf('\n---------------------------------------------\n');

fprintf('RAW SENSOR + LQR\n');
fprintf('---------------------------------------------\n');

fprintf('Landing time       = %.3f s\n',off_time);
fprintf('Landing X          = %.6f m\n',off_X);
fprintf('Landing Y          = %.6f m\n',off_Y);
fprintf('Landing Z          = %.6f m\n',off_Z);
fprintf('Horizontal error   = %.6f m\n',off_error);

fprintf('\n---------------------------------------------\n');

fprintf('KF + LQR\n');
fprintf('---------------------------------------------\n');

fprintf('Landing time       = %.3f s\n',kf_time);
fprintf('Landing X          = %.6f m\n',kf_X);
fprintf('Landing Y          = %.6f m\n',kf_Y);
fprintf('Landing Z          = %.6f m\n',kf_Z);
fprintf('Horizontal error   = %.6f m\n',kf_error);

fprintf('\n---------------------------------------------\n');

fprintf('KF effect:\n');
fprintf('Change in landing error = %.6f m\n', ...
    kf_error-off_error);

fprintf('Improvement = %.2f %%\n',improvement);

fprintf('=============================================\n');