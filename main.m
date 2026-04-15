% 13/4/26 Victor Emmanuel Meimaris: Created main.m script to recreate the results from 
% "Phase-coded FMCW Laser Headlamp for Integrated Sensing, Communication, and Illumination"
% This function handles setting the real parameters and plotting the results using the paper's methods.


%------------Notes----------
% [1]* Phase code is calculate this way because #Rb*T random bits of 0 and 1s
% will be allocated to the matrix and then mulltiplying this 1D array with
% 2, gives 0s and 2s which subtracted by 1 give -1 and 1s need for the
% phase codes exponentials which give 0,π phases.



clc;
clear;

% ----------Assumptions-----
% 1)The FOV of the Camera is 360, if not, a simple if loop does not take into account
% the echos created by a car outside the FOV.
%
% 2)




% Initial Parameters as used in the paper.

fc = 193.4e12; % Laser/Carrier Frequency fc = 193.4 Thz
B = 10e9; % Chirp Bandwidth B = 10Ghz
T_chirp = 10e-6; % Chirp Period T = 10μs
Rb = 1e9; % Data rate Rb = 1Gbps
Ts = 1 / Rb; % Symbol duration
m_slope = B / T_chirp; % slope μ
fs = 2 * Rb; % Sampling frequency for the chirp using Nyquist theorem
M = 256; % Number of chirps per frame

% Simulation clocks
t_global = 0 :0.1: 3; % Global clock for the kinematic calculations of x(t), y(t), R(t), theta(t) and Ur(t).dt = 0.1s is enough for the speeds used by cars.
t_chirp = (0: 1/fs: T_chirp - 1/fs)'; % Chirp clock propagating the signals, t[n] = nTsample where ne[0,N-1] and Tsample = 1/fs.
t_frame = 0: T_chirp: (M-1) * T_chirp; % Frame clock 

% --------------Real world parameters for the simulation--------------

% The number of elements in the targets indicates the number of cars and the
% index of each car must be the same for every matrix!

x0 = [-3.5, 0]; % Target starting position x0 in meters.
y0 = [5, 30]; % Target starting position y0 in meters.

ux0 = [0, 0]; % Target starting velocity on x in m/s
uy0 = [-27.78, 13.89]; % Target starting velocity on y in m/s

a_x = [0, 0]; % Target accelaration on x in m/s^2.
a_y = [-2.5, 0]; % Target accelaration on y in m/s^2.

% Vehicles

% Cars with basic trajectiories using the above matrices
x = x0 + ux0 .* t_global' + 1/2 * a_x .* t_global'.^2;
y = y0 + uy0 .* t_global' + 1/2 * a_y .* t_global'.^2;

% Cars with complex trajectories like an overtake
x1 = 0 - 1/2 * 3.5* cos(pi.* t_global'/6) .*t_global'.^2;
y1 = -15 + 3 * t_global' + 1/2 * 4 * sin(pi* t_global'/6) .*t_global'.^2;

% Fusing basic trajectories and complex to the matrices x and y
x = [x, x1];
y = [y, y1];
% Calculating given camera angle theta at that time for each vehicle
theta = atan2d(y, x);



% Will be put in the loop
%phase_code = 2 * randi([0 1], 1, Rb * T) - 1; % *[1] Phase code for the ego car, 10kbits since 1Gbps * 10μs for the chirp period and the data rate.
