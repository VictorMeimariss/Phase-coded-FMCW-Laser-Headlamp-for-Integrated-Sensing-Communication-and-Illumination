% 13/4/26 Victor Emmanuel Meimaris: Created main.m script to recreate the results from 
% "Phase-coded FMCW Laser Headlamp for Integrated Sensing, Communication, and Illumination"
% This function handles setting the real parameters and plotting the results using the paper's methods.


%------------Notes----------


clc;
clear;

% ----------Assumptions-----
% 1)The FOV of the Camera is 360, if not, a simple if loop does not take into account
% the echos created by a car outside the FOV.
%
% 2)




% Initial Parameters as used in the paper.

t_end = 3; % Ending time of sim in seconds
dt_global = 0.1; % Global time interval
c = 299702547; % Speed of light in air in m/s
fc = 193.4e12; % Laser/Carrier Frequency fc = 193.4 Thz
B = 10e9; % Chirp Bandwidth B = 10Ghz
T_chirp = 10e-6; % Chirp Period T = 10μs
Rb = 1e9; % Data rate Rb = 1Gbps
fs = Rb; % This enough sampling frequency and it is explained why in the generation of the beat signal
m_slope = B / T_chirp; % slope μ
M = 256; % Number of chirps per frame

% Simulation clocks
t_global = 0 :dt_global: t_end; % Global clock for the kinematic calculations of x(t), y(t), R(t), theta(t) and Ur(t).dt = 0.1s is enough for the speeds used by cars.
t_chirp = 0: 1/fs: T_chirp - 1/fs; % Chirp clock propagating the signals, t[n] = nTsample where ne[0,N-1] and Tsample = 1/fs.

% --------------Real world parameters for the simulation--------------

SNR = 20; % Signal to noise ratio for simulating noise.
num_interf = 0; % Number of vehicles that interfere with the signal

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



% Now every variable is calculated and the loop starts, the loop simulates
% each time interval to use the real world information and simulate the
% system
for i = 1:1%size(x, 1)
    % A beat signal array needs to be generated using the variables, it is the result
    % of the noisy echos as well as interference from other car echos mixed
    % with the original signal. Once the beat_signal matrix is generated, a Group Delay Filter
    % needs to be applied. This cancels the phase coding, restoring the linear frequency
    % modulation (LFM) structure and improving range-Doppler estimation accuracy
    signal = generate_signal(t_chirp, M, m_slope, T_chirp, Rb, fc, SNR, num_interf, fD(i, :), tau(i, :));

    % Once the beat_signal matrix is generated, a Group Delay Filter needs to be
    % applied. This cancels the phase coding, restoring the linear frequency
    % modulation (LFM) structure and improving range-Doppler estimation accuracy
    
end