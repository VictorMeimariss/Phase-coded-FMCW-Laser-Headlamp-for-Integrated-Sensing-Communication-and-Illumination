% This script recreates the test scenarios from Section IV-C of the paper.

% Scenario 1: a 100x100 2D space contains two linear tracks
% corrupted by Gaussian noise and random clutter.

clc; clear; close all;

figure('Name', 'MHT-based TBD Validation (Paper Figure 4)', 'Position', [200, 100, 1000, 600]);

num_points = 500;
noise_scalar = 0.1;
gaussian_noise = randn(num_points, 1) * noise_scalar;
num_clutter = 100;
c_x = 0 + rand(num_clutter, 1) * 100;
c_y = 0 + rand(num_clutter, 1) * 100;
c_t = 0 + rand(num_clutter, 1) * 10;

t = linspace(0, 10, num_points)';

x1_true = 10 + 8*t;
y1_true = x1_true;

x1 = x1_true + gaussian_noise;
y1 = x1;

x2_true = x1_true;
y2_true = 90 - 8*t;

x2 = x2_true + gaussian_noise;
y2 = 90 - 8*t + gaussian_noise;

point_cloud = [
    x1, y1, t;
    x2, y2, t;
    c_x, c_y, c_t;
];

tracks = MHT_Track_Detection( ...
    point_cloud, ...
    'window_length', 2, ...
    'gap', 1, ...
    'stitch_threshold', 5, ...
    'num_of_peaks', 10 ...
);

subplot(2, 3, 1); % fig 4(a)

hold on; grid on;
xlabel('X Position (m)'); ylabel('Y Position (m)'); zlabel('Time (s)'); title('(a)');

scatter3(c_x, c_y, c_t, 10, [0.8 0.8 0.8], 'filled', 'DisplayName', 'clutter');

plot3(x1_true, y1_true, t, 'g--', 'LineWidth', 3, 'HandleVisibility', 'off');  % seperating them because plot
plot3(x2_true, y2_true, t, 'g--', 'LineWidth', 3, 'DisplayName', 'True Track');% combines joins the lines together

x = tracks{1}.points(:,1); y = tracks{1}.points(:,2); t = tracks{1}.points(:,3);
plot3(x, y, t, 'b:', 'LineWidth', 2, 'HandleVisibility', 'off');
x = tracks{2}.points(:,1); y = tracks{2}.points(:,2); t = tracks{2}.points(:,3);
plot3(x, y, t, 'b:', 'LineWidth', 2, 'DisplayName', 'Detected Track');

hold off;
axis([-20 100 -40 100]);
legend('Location', 'southwest');

% generating binary image for the xy plane
BW = false(101, 101);

x = point_cloud(:,1); y = point_cloud(:,2);

gridres = 1;
col_idx = round(x)+1;
row_idx = round(y)+1;

linear_indices = sub2ind(size(BW), row_idx, col_idx);
BW(linear_indices) = true;

[H, theta, rho] = hough(BW);

subplot(2, 3, 2); % fig 4(b)
imshow(imadjust(rescale(H)),'XData',theta,'YData',rho,'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho'); title('(b)');
axis on, axis normal, hold on;
colormap(gca,hot);

subplot(2, 3, 3); % fig 4(c)

mean_filter = fspecial('average', [3 3]);
H_filtered = round(imfilter(H, mean_filter));

imshow(imadjust(rescale(H_filtered)),'XData',theta,'YData',rho,'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho'); title('(c)');
axis on, axis normal, hold on;
colormap(gca,hot);

peaks = houghpeaks(H_filtered, 4, 'threshold', ceil(0.3 * max(H_filtered(:))));

subplot(2, 3, 4); hold on; % fig 4(d)
imshow(imadjust(rescale(H_filtered)),'XData',theta,'YData',rho,'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho'); title('(d)');
axis on, axis normal, hold on;
colormap(gca,hot);
plot(theta(peaks(:,2)), rho(peaks(:,1)), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
legend('Detected Peaks');


% Scenario 2: one linear and one non-linear tracks embedded in dense clutter

num_points = 500;
num_clutter = 100;
c_x = 0 + rand(num_clutter, 1) * 100;
c_y = 0 + rand(num_clutter, 1) * 100;
c_t = 0 + rand(num_clutter, 1) * 10;

t = linspace(0, 10, num_points)';

x1 = 10 + 8*t;
y1 = 30 + 6*t;

x2 = 20 + 7*t;
y2 = -0.9 * (t - 7).^2 + 60;

subplot(2, 3, 5); % fig 4(e)

hold on; grid on;
xlabel('X Position (m)'); ylabel('Y Position (m)'); zlabel('Time (s)'); title('(e)');

scatter3(c_x, c_y, c_t, 10, [0.8 0.8 0.8], 'filled', 'DisplayName', 'clutter');
plot3(x1, y1, t, 'g--', 'LineWidth', 2, 'DisplayName', 'Original Points 1');
plot3(x2, y2, t, 'y--', 'LineWidth', 2, 'DisplayName', 'Original Points 2');

axis([0 100 0 100]);
legend('Location', 'southeast');

subplot(2, 3, 6); % fig 4(f)

hold on; grid on;
xlabel('X Position (m)'); ylabel('Y Position (m)'); zlabel('Time (s)'); title('(f)');

point_cloud = [
    x1, y1, t;
    x2, y2, t;
    c_x, c_y, c_t;
];

tracks = MHT_Track_Detection(point_cloud);

scatter3(c_x, c_y, c_t, 10, [0.8 0.8 0.8], 'filled', 'DisplayName', 'clutter');
x = tracks{1}.points(:,1); y = tracks{1}.points(:,2); t = tracks{1}.points(:,3);
plot3(x, y, t, 'b--', 'LineWidth', 2, 'DisplayName', 'Detected Track 2');
x = tracks{2}.points(:,1); y = tracks{2}.points(:,2); t = tracks{2}.points(:,3);
plot3(x, y, t, 'r--', 'LineWidth', 2, 'DisplayName', 'Detected Track 1');

axis([0 100 0 100]);
legend('Location', 'southeast');