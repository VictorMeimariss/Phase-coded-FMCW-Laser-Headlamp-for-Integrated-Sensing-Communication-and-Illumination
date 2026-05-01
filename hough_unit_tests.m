% These are a number of unit tests to make sure the performance
% of the HT transform is on par while experimenting.

%% one simple target
clc; clear;

num_points = 500;
noise_scalar = 0.1;
gaussian_noise = randn(num_points, 1) * noise_scalar;
num_clutter = 100;
c_x = 0 + rand(num_clutter, 1) * 100;
c_y = 0 + rand(num_clutter, 1) * 100;
c_t = 0 + rand(num_clutter, 1) * 10;

t = linspace(0, 10, num_points)';

x1 = 10 + 5*t;
y1 = x1;

point_cloud = [
    x1, y1, t;
    c_x, c_y, c_t;
];

tracks = MHT_Track_Detection(point_cloud, 'num_of_peaks', 10);

plot_tracks(tracks, c_x, c_y, c_t);

%% 3 simple targets with gaussian noise
clc; clear;

num_points = 500;
noise_scalar = 0.1;
gaussian_noise = randn(num_points, 1) * noise_scalar;
num_clutter = 0;
c_x = 0 + rand(num_clutter, 1) * 100;
c_y = 0 + rand(num_clutter, 1) * 100;
c_t = 0 + rand(num_clutter, 1) * 10;

t = linspace(0, 10, num_points)';

x1 = 10 + 5*t;
y1 = x1;

x2 = 10 + 8*t;
y2 = 40 + 0*t + gaussian_noise;

x3 = 0 + 7*t;
y3 = 0.9 * (t - 5).^2 + 70 + gaussian_noise;

point_cloud = [
    x1, y1, t;
    x2, y2, t;
    x3, y3, t;
    c_x, c_y, c_t;
];

tracks = MHT_Track_Detection(point_cloud, 'num_of_peaks', 10);

plot_tracks(tracks, c_x, c_y, c_t);

%% 2 targets crossing paths with dense clutter
clc; clear;

num_points = 500;
noise_scalar = 0;
gaussian_noise = randn(num_points, 1) * noise_scalar;
num_clutter = 200;
c_x = 0 + rand(num_clutter, 1) * 100;
c_y = 0 + rand(num_clutter, 1) * 100;
c_t = 0 + rand(num_clutter, 1) * 10;

t = linspace(0, 10, num_points)';

x1 = 10 + 8*t;
y1 = x1;

x2 = 10 + 8*t;
y2 = 90 - 8*t;

point_cloud = [
    x1, y1, t;
    x2, y2, t;
    c_x, c_y, c_t;
];

tracks = MHT_Track_Detection( ...
    point_cloud, ...
    'window_length', 2, ...
    'gap', 0.5, ...
    'stitch_threshold', 5, ...
    'num_of_peaks', 10 ...
);

plot_tracks(tracks, c_x, c_y, c_t);

%% two stopped cars
clc; clear;

num_points = 500;
noise_scalar = 0.1;
gaussian_noise = randn(num_points, 1) * noise_scalar;
num_clutter = 0;
c_x = 0 + rand(num_clutter, 1) * 100;
c_y = 0 + rand(num_clutter, 1) * 100;
c_t = 0 + rand(num_clutter, 1) * 10;

t = linspace(0, 10, num_points)';

x1 = 45 + 0*t;
y1 = x1;

x2 = 50 + 0*t;
y2 = x2;

point_cloud = [
    x1, y1, t;
    x2, y2, t;
    c_x, c_y, c_t;
];

tracks = MHT_Track_Detection(point_cloud);

plot_tracks(tracks, c_x, c_y, c_t);

view(3);

%% target scenario from main.m (1/2)
clc; clear;

num_points = 31;

t = linspace(0, 3, num_points);

x0 = [-3.5, 0];
y0 = [5, 30];

ux0 = [0, 0];
uy0 = [-27.78, 13.89];

a_x = [0, 0];
a_y = [-2.5, 0];

x = x0 + ux0 .* t' + 1/2 * a_x .* t'.^2;
y = y0 + uy0 .* t' + 1/2 * a_y .* t'.^2;

x1 = 0 - 1/2 * 3.5 * cos(pi * t'/(t(end) * 2)) .*t'.^2;
y1 = -15 + 3 * t' + 1/2 * 4 * sin(pi * t'/(t(end) * 2)) .*t'.^2;

x = [x, x1];
y = [y, y1];

t = repmat(t', 1, 3);

point_cloud = [];

tracks = {};

figure('Name', '3D Track Mapping');
hold on; grid on;
xlabel('X Position (m)'); ylabel('Y Position (m)'); zlabel('Time (s)');
title('Target Trajectories in Spatio-Temporal Space');
xlim([-5 5]);
ylim([-25 75]);
plot(0, 0, 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'ego car');

detection_rate = 0.5;

for i=1:size(t,1)
    point_cloud = [point_cloud; [x(i,:)', y(i,:)', t(i,:)']];
    
    pause(0.15);
    
    if mod(t(i,1),detection_rate) ~= 0 && t(i,1) ~= 0; continue; end

    tracks = MHT_Track_Detection(...
        point_cloud, ...
        'tracks', tracks, ...
        'window_length', 0.4, ...
        'num_of_peaks', 10, ...
        'minimum_common_points', 2, ...
        'gap', 3 ...
    );

    for j = 1:length(tracks)
        pts_x = tracks{j}.points(:,1);
        pts_y = tracks{j}.points(:,2);
        pts_t = tracks{j}.points(:,3);

        plot3(pts_x, pts_y, pts_t, 'LineWidth', 2);
    end

    point_cloud = [];
end

%% target scenario from main.m (2/2)

clc; clear;

num_points = 31;
noise_scalar = 0;
gaussian_noise = randn(num_points, 1) * noise_scalar;
num_clutter = 0;
c_x = -5 + rand(num_clutter, 1) * 10;
c_y = -20 + rand(num_clutter, 1) * 95;
c_t = 0 + rand(num_clutter, 1) * 10;

t = linspace(0, 3, num_points);

x0 = [-3.5, 0];
y0 = [5, 30];

ux0 = [0, 0];
uy0 = [-27.78, 13.89];

a_x = [0, 0];
a_y = [-2.5, 0];

x = x0 + ux0 .* t' + 1/2 * a_x .* t'.^2;
y = y0 + uy0 .* t' + 1/2 * a_y .* t'.^2;

x1 = 0 - 1/2 * 3.5 * cos(pi * t'/(t(end) * 2)) .*t'.^2;
y1 = -15 + 3 * t' + 1/2 * 4 * sin(pi * t'/(t(end) * 2)) .*t'.^2;

x = [x, x1];
y = [y, y1];

point_cloud = [
    x(:), y(:), repmat(t',3,1);
    c_x, c_y, c_t;
];

tracks = MHT_Track_Detection(...
    point_cloud, ...
    'window_length', 0.4, ...
    'num_of_peaks', 10, ...
    'minimum_common_points', 2, ...
    'stitch_threshold', 5.0, ...
    'gap', 3 ...
);

plot_tracks(tracks, c_x, c_y, c_t);

xlim([-5 5]);
ylim([-25 75]);

plot(0, 0, 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'ego car');

%% local function for drawing tracks in three dimensions

function plot_tracks(tracks, c_x, c_y, c_t)  
    disp(['Total tracks detected: ', num2str(length(tracks))]);

    figure('Name', '3D Track Mapping');
    hold on; grid on;
    xlabel('X Position (m)'); ylabel('Y Position (m)'); zlabel('Time (s)');
    title('Target Trajectories in Spatio-Temporal Space');
    xlim([-10 100])
    ylim([-10 100])

    for idx = 1:length(tracks)
        x = tracks{idx}.points(:,1);
        y = tracks{idx}.points(:,2);
        t = tracks{idx}.points(:,3);
        
        plot3(x, y, t, 'LineWidth', 2, 'DisplayName', ['Target ', num2str(idx)]);
        %scatter3(x, y, t, 'LineWidth', 2, 'DisplayName', ['Target ', num2str(idx)]); 

    end
    
    scatter3(c_x, c_y, c_t, 10, [0.75 0.75 0.75], 'filled', 'DisplayName', 'clutter');

    %view(3); % Set to default 3D perspective
    legend('show', 'Location', 'northeastoutside');
    
end