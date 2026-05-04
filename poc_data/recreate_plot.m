clc; clear; close all;

addpath .. % need for ADB.m

load('num_of_peaks_30_min_pts_5/input_data.mat')
load('num_of_peaks_30_min_pts_5/tracks.mat')

figure('Position', [180, 200, 600, 500]); hold; grid on;
axis([-5 5 -90 72]); legend('Location', 'northeast');
plot(0, 0, 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'ego car');
x = input_data(:,1);
y = input_data(:,2);
t = input_data(:,3);
plot3(x, y, t, 'g.', 'DisplayName', 'input data');


h2 = figure('Position', [800, 200, 600, 500]); hold; grid on;
axis([-5 5 -90 72]); legend('Location', 'northeast');
plot(0, 0, 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'ego car');


% lights figure
h3 = figure;
ax = axes(h3);
set(ax, 'ColorOrder', repelem(get(ax, 'ColorOrder'), 4, 1), 'NextPlot', 'replacechildren');
hold on; grid on;
axis([0 3 -20 20]);
set(gca, 'YDir', 'reverse'); % reverse the vertical axis
xlabel('Time (s)'); ylabel('Shadow Angle (^{\circ})'); zlabel('Light Intensity');
title('(a) oncoming vehicle');
% view angle limits
yline(-15, 'k-', 'LineWidth', 1.2, 'HandleVisibility', 'off'); 
yline(15, 'k-', 'LineWidth', 1.2, 'HandleVisibility', 'off');

for i=1:length(tracks)
    figure(h2);
    x = tracks{i}.points(:,1);
    y = tracks{i}.points(:,2);
    t = tracks{i}.points(:,3);
    plot3(x, y, t, '.', 'DisplayName', ['Target ', num2str(i)]);
    
    
    figure(h3);
    
    illumination_data = zeros(length(t), 5);
    
    for j=1:length(t)
        illumination_data(j,:) = ADB(x(j),y(j));
    end
    
    left_thetaL = illumination_data(:,1);
    left_thetaR = illumination_data(:,2);
    right_thetaL = illumination_data(:,3);
    right_thetaR = illumination_data(:,4);
    intensity = illumination_data(:,5);

    % left lamp left/right theta boundaries
    plot3(t, left_thetaL, intensity, '--', 'LineWidth', 1);
    plot3(t, left_thetaR, intensity, '--', 'LineWidth', 1);
    % right lamp left/right boundaries
    plot3(t, right_thetaL, intensity, '-', 'LineWidth', 1);
    plot3(t, right_thetaR, intensity, '-', 'LineWidth', 1);
end

rmpath ..