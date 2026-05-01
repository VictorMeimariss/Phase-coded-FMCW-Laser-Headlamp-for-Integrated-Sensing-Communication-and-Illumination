clc; clear; close all;

load('dt_0_01_sec/input_data.mat')
load('dt_0_01_sec/tracks.mat')

figure; hold; grid on; axis([-5 5 -90 72]); legend('Location', 'northeast');

plot(0, 0, 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'ego car');

plot3(input_data(:,1),input_data(:,2),input_data(:,3), 'go', 'MarkerSize', 0.7, 'DisplayName', 'input data');

for i=1:length(tracks)
    x = tracks{i}.points(:,1);
    y = tracks{i}.points(:,2);
    t = tracks{i}.points(:,3);
    plot3(x, y, t, 'LineWidth', 2, 'DisplayName', ['Target ', num2str(i)]);
end