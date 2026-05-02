clc; clear;

load('num_of_peaks_30_min_pts_5/input_data.mat')
load('num_of_peaks_30_min_pts_5/tracks.mat')

figure('Position', [180, 200, 600, 500]); hold; grid on;
axis([-5 5 -90 72]); legend('Location', 'northeast');
plot(0, 0, 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'ego car');
x = input_data(:,1);
y = input_data(:,2);
t = input_data(:,3);
plot3(x, y, t, 'g.', 'DisplayName', 'input data');


figure('Position', [800, 200, 600, 500]); hold; grid on;
axis([-5 5 -90 72]); legend('Location', 'northeast');
plot(0, 0, 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'ego car');

for i=1:length(tracks)
    x = tracks{i}.points(:,1);
    y = tracks{i}.points(:,2);
    t = tracks{i}.points(:,3);
    plot3(x, y, t, '.', 'DisplayName', ['Target ', num2str(i)]);
end

