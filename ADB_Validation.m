% This script recreates the test scenarios from Section IV-B of the paper.
clear; clc; close all;

lane_width = 3.5; % average lane width in meters

% scenario 1: ego vehicle at 40 km/h, oncoming at 30 km/h, initial distance 150m
v_ego = 40 / 3.6;     % km/h to m/s
v_oncoming = -30 / 3.6;
v_oncomming_rel = v_ego - v_oncoming;

t = linspace(0, 7, 100);

x = -lane_width +  0 * t;
y = 150 - v_oncomming_rel * t;

illumination_data = zeros(length(t),5);

for i=1:length(t)
    illumination_data(i, :) = ADB(x(i),y(i));
end

left_thetaL = illumination_data(:,1);
left_thetaR = illumination_data(:,2);
right_thetaL = illumination_data(:,3);
right_thetaR = illumination_data(:,4);
intensity = illumination_data(:,5);

% plotting figure 3(a)
figure('Name', 'ADB shadow angle adjustment (with real dimensions)', 'Position', [100, 100, 1000, 400]);

subplot(1, 2, 1); hold on; axis([0 7 -20 20]); grid on;
set(gca, 'YDir', 'reverse'); % reverse the vertical axis
xlabel('Time (s)'); ylabel('Shadow Angle (^{\circ})'); zlabel('Light Intensity');
title('(a) oncoming vehicle');

% view angle limits
yline(-15, 'k-', 'LineWidth', 1.2, 'HandleVisibility', 'off'); 
yline(15, 'k-', 'LineWidth', 1.2, 'HandleVisibility', 'off');

% left lamp left/right theta boundaries
plot3(t, left_thetaL, intensity, 'b--', 'LineWidth', 1, 'DisplayName', 'Left thetaL');
plot3(t, left_thetaR, intensity, 'b-', 'LineWidth', 1, 'DisplayName', 'Left thetaR');
% right lamp left/right boundaries
plot3(t, right_thetaL, intensity, 'r--', 'LineWidth', 1, 'DisplayName', 'Right thetaL');
plot3(t, right_thetaR, intensity, 'r-', 'LineWidth', 1, 'DisplayName', 'Right thetaR');

legend('Location', 'southwest');

% scenario 2: ego vehicle at 50 km/h, following two vehicles, initial distance 30m
v_ego = 50 / 3.6; 

% to create the curve in 3b we assume the car on the left has the same
% speed, while the car on the right is quite slower
v_left = v_ego;
v_right = 25 / 3.6;

v_left_rel = v_ego - v_left;
v_right_rel = v_ego - v_right;

t = linspace(0, 2, 100);

x1 = -lane_width + 0*t;
y1 = 30 - v_left_rel * t;

x2 = lane_width + 0*t;
y2 = 30 - v_right_rel * t;


idata1 = zeros(length(t),5);
idata2 = idata1;

for i=1:length(t)
    idata1(i, :) = ADB(x1(i),y1(i));
    idata2(i, :) = ADB(x2(i),y2(i));
end

% plotting figure 3(b)
subplot(1, 2, 2); hold on; axis([0 2 -20 20]); grid on;
set(gca, 'YDir', 'reverse'); % reverse the vertical axis
xlabel('Time (s)'); ylabel('Shadow Angle (^{\circ})'); zlabel('Light Intensity');
title('(b) multiple preceding vehicles');

% view angle limits
yline(-15, 'k-', 'LineWidth', 1.2); 
yline(15, 'k-', 'LineWidth', 1.2);

% ----- car on the left lane -----
left_thetaL = idata1(:,1);
left_thetaR = idata1(:,2);
right_thetaL = idata1(:,3);
right_thetaR = idata1(:,4);
intensity = idata1(:,5);

% left lamp left/right theta boundaries
plot3(t, left_thetaL, intensity, 'g--', 'LineWidth', 1);
plot3(t, left_thetaR, intensity, 'g-', 'LineWidth', 1);
% right lamp left/right boundaries
plot3(t, right_thetaL, intensity, 'c--', 'LineWidth', 1);
plot3(t, right_thetaR, intensity, 'c-', 'LineWidth', 1);

% ----- car on the right lane -----
left_thetaL = idata2(:,1);
left_thetaR = idata2(:,2);
right_thetaL = idata2(:,3);
right_thetaR = idata2(:,4);
intensity = idata2(:,5);

% left lamp left/right theta boundaries
plot3(t, left_thetaL, intensity, 'r--', 'LineWidth', 1);
plot3(t, left_thetaR, intensity, 'r-', 'LineWidth', 1);
% right lamp left/right boundaries
plot3(t, right_thetaL, intensity, 'b--', 'LineWidth', 1);
plot3(t, right_thetaR, intensity, 'b-', 'LineWidth', 1);