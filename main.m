% 13/4/26 Victor Emmanuel Meimaris: Created main.m script to recreate the results from 
% "Phase-coded FMCW Laser Headlamp for Integrated Sensing, Communication, and Illumination"
% This function handles setting the real parameters and plotting the results using the paper's methods.


%------------Notes----------


clc;
clear;
tic;
% ----------Assumptions-----
% 1)The FOV of the Camera is 360, if not, a simple if loop does not take into account
% the echos created by a car outside the FOV.
%
% 2) SNR is taken as an average for all cars, in reality the SNR is better
% on closer cars than on ones which are further.
%
% 3) The interferences are simulated as random each iteration since the
% phase coding even in the real world makes those signals random each time
% iteration, since the interferences seem random, the thetas from the camera
% each iteration are also random. Hence why in this code, the real thetas
% of the cars will be used and random ones will be assigned to the ghost
% cars. In reality the theta is calculated based on the number of chirps
% and which chirp was used when sending the signal, that chirp has a
% specific theta which is assigned to the signal, knowing the indices of
% the RND, after the CA-CFAR, the thetas are assigned to each range. But in
% the real world a filter is used to fix the theta. This could be simulated
% as well but it would take too much time and the results would be
% identical.



% Initial Parameters as used in the paper.

t_end = 3; % Ending time of sim in seconds
dt_global = 0.1; % Global time interval
c = 299702547; % Speed of light in air in m/s
fc = 193.4e12; % Laser/Carrier Frequency fc = 193.4 Thz
B = 10e9; % Chirp Bandwidth B = 10Ghz
T_chirp = 10e-6; % Chirp Period T = 10μs
Rb = 1e9; % Data rate Rb = 1Gbps
fs = 4* Rb; % NOT ->This enough sampling frequency and it is explained why in the generation of the beat signal
N = fs * T_chirp; % Number of samples
m_slope = B / T_chirp; % slope μ
M = 256; % Number of chirps per frame


% Simulation clocks
t_global = 0 :dt_global: t_end; % Global clock for the kinematic calculations of x(t), y(t), R(t), theta(t) and Ur(t).dt = 0.1s is enough for the speeds used by cars.
t_chirp = (0:N-1)/fs; % Chirp clock propagating the signals, t[n] = nTsample where ne[0,N-1]

% --------------Real world parameters for the simulation--------------

SNR = -14; % Signal to noise ratio for simulating noise
num_interf = 10; % Number of vehicles that interfere with the signal

% CA-CFAR variables
if SNR <= -19 % This one I will test in combination with the hough transform, maybe it wont be needed, because it takes double the time
    guard_size = [8 4];
    training_size = [16 8];
    pfa = 1e-3;
else
    guard_size = [4 2];
    training_size = [8 4];
    pfa = 1e-4; % I can add another elseif with the same sizes but stricter pfa for higher snrs, but i want to test the hough transform
end


% The number of elements in the targets indicates the number of cars and the
% index of each car must be the same for every matrix!

x0 = [-3.5, 0]; % Target starting position x0 in meters.
y0 = [5, 30]; % Target starting position y0 in meters.

ux0 = [0, 0]; % Target starting velocity on x in m/s
uy0 = [-27.78, 13.89]; % Target starting velocity on y in m/s

a_x = [0, 0]; % Target accelaration on x in m/s^2.
a_y = [-2.5, 0]; % Target accelaration on y in m/s^2.

% Vehicles

% If you want to add a basic trajectory vehicle just add to the
% above matrices, else write down your own x2...xn, ...yn, concatenate
% them to x and y, calculate ux and uy,(either manually, or with gradient
% which is not reccomended because it doesnt give accurate results)
% concantanate the results to the ux and uy matrices as well and you are good to go

% Cars with basic trajectiories using the above matrices
x = x0 + ux0 .* t_global' + 1/2 * a_x .* t_global'.^2;
y = y0 + uy0 .* t_global' + 1/2 * a_y .* t_global'.^2;

% Cars with complex trajectories like an overtake
x1 = 0 - 1/2 * 3.5 * cos(pi * t_global'/(t_end * 2)) .*t_global'.^2;
y1 = -15 + 3 * t_global' + 1/2 * 4 * sin(pi * t_global'/(t_end * 2)) .*t_global'.^2;

% Concatenating basic trajectories and complex to the matrices x and y
x = [x, x1];
y = [y, y1];

% Calculation of given camera angle theta at that time for each vehicle
theta = atan2d(y, x);

% Cartesian speed, manual derivatives for less error
ux = ux0 + a_x.* t_global';
uy = uy0 + a_y.* t_global';

% This was painfull...
ux1 = -1.75 .* (2 .* t_global' .* cos(pi * t_global'/6) - (pi/6) .* (t_global'.^2) .* sin(pi * t_global'/6));
uy1 = 3 + (pi/3) .* cos(pi * t_global'/6) .* (t_global'.^2) + 4 .* t_global' .* sin(pi * t_global'/6);

ux = [ux, ux1];
uy = [uy, uy1];

% Range from the ego car
R = sqrt(x.^2 + y.^2);
tau = 2 * R ./ c; % Signal propagation delay

% Doppler frequency calculation using the radial speed
ur = (x .* ux + y .* uy) ./ R;
fD = 2 * ur * fc / c;

% needed for HT
point_cloud = [];
tracks = {};
track_detection_rate = 0.5; % every this many seconds send our data for
                            % track detection, be careful when changing
                            % this since the algorithm inputs have been
                            % tweaked to work for this exact value (due the
                            % limited number of points)
input_data = []; % storing the input data to plot later

% Now every variable is calculated and the loop starts, the loop simulates
% each time interval to use the real world information and simulate the
% system

% displaying the trajectory data in real time
figure('Name', '3D Track Mapping');
hold on; grid on;
xlabel('X Position (m)'); ylabel('Y Position (m)'); zlabel('Time (s)');
title('Target Trajectories in Spatio-Temporal Space');
xlim([-5 5]);
ylim([-25 75]);
plot(0, 0, 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'ego car');
drawnow; % forces figure to update (matlab only draws at the end when doing heavy processing)

for i = 1:length(t_global)
    % A beat signal array needs to be generated using the variables, it is the result
    % of the noisy echos as well as interference from other car echos mixed
    % with the original signal. Once the beat_signal matrix is generated, a Group Delay Filter
    % needs to be applied. This cancels the phase coding, restoring the linear frequency
    % modulation (LFM) structure and improving range-Doppler estimation accuracy

    % Once the beat_signal matrix is generated, a Group Delay Filter needs to be
    % applied. This cancels the phase coding, restoring the linear frequency
    % modulation (LFM) structure and improving range-Doppler estimation accuracy
    
    signal = generate_signal(t_chirp, M, m_slope, T_chirp, Rb, fs, N, fc, SNR, num_interf, fD(i, :), tau(i, :));

    % Extracting coordinates using the decouple_signal function which uses
    % the signal with 2D fast fourier transform and then ca-cfar to
    % extract the range, then the assigned thetas as well as random ones on
    % the ghosts are assigned and physical coordinates x, y are extracted
    % in the form of 2 collumns [x y].

    %tic;
    coordinates = decouple_signal(M, N, T_chirp, signal, fs, c, B, guard_size, training_size, pfa, theta(i,:), R(i,:));
    %elapsedTime = toc;
    
    % The coordinates plus the time are passed to the hough
    % transform which evaluates if some signals, are ghost, or real.
    
    time = t_global(i) .* ones(size(coordinates, 1), 1);
    detected_points = [coordinates time];
    
    input_data = [input_data; detected_points];
    
    point_cloud = [point_cloud; detected_points];
    
    disp(t_global(i));
    
    if mod(t_global(i),track_detection_rate) ~= 0 || t_global(i) == 0; continue; end
    
    % M being only 2 makes the algorithm very susceptible to noise, however,
    % if you go in hough_unit_tests.m and increase the frame rate for this
    % same scenario you can see that clutter isn't a problem
    
    % since the loop takes a while to run we've saved the results from a run
    % under poc_data/
    
    tracks = MHT_Track_Detection(...
        point_cloud, ...
        'tracks', tracks, ...
        'window_length', 0.4, ...
        'num_of_peaks', 10, ...
        'minimum_common_points', 2, ...
        'gap', 2.9 ...
    );

    for j = 1:length(tracks)
        pts_x = tracks{j}.points(:,1);
        pts_y = tracks{j}.points(:,2);
        pts_t = tracks{j}.points(:,3);

        plot3(pts_x, pts_y, pts_t, 'LineWidth', 2);
    end
    
    scatter3(point_cloud(:,1),point_cloud(:,2),point_cloud(:,3),'LineWidth',2);
    
    drawnow; % forces figure to update

    point_cloud = []; % cleaning the point_cloud "buffer"
end
total_elapsed_time = toc;

hold off;

% % Plot test 
% final_spectrum = abs(fftshift(fft(signal(:, 256)), 1));
% 
% % Plotting
% f_axis = linspace(-fs/2, fs/2 - fs/N, N);
% figure;
% plot(f_axis/1e6, final_spectrum);
% xlabel('Frequency (MHz)'); ylabel('Magnitude'); title('Range Peaks After GDF + Decoding'); 