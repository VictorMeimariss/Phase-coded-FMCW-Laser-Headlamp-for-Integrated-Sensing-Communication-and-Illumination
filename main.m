% 13/4/26 Victor Emmanuel Meimaris: Created main.m script to recreate the results from 
% "Phase-coded FMCW Laser Headlamp for Integrated Sensing, Communication, and Illumination"
% This function handles setting the real parameters and plotting the results using the paper's methods.

clc;
clear;

% Assumptions: 
% 1)The FOV of the Camera is 360, if not, a simple if loop does not take into account
% the echos created by a car outside the FOV.

% Time intervals

t_global = 0: 0.1:5; % Global time for the kinematic calculations of x(t), y(t), R(t), θ(t) and ur(t).

% Initial Parameters as used in the paper.

fc = 193.4e12; % Laser/Carrier Frequency fc = 193.4 Thz
B = 10e9; % Chirp Bandwidth B = 10Ghz
T = 10e-6; % Chirp Period T = 10μs
Rb = 1e9; % Data rate Rb = 1Gbps
Ts = 1 / Rb; % Symbol duration


% --------------Changing parameters for the simulation--------------

% The number of elements in the targets indicates the number of cars and the
% index of each car must be the same for every matrix!

% Example to test a car in the front and a car to the left
% speeding, one vehicle is coming from the other lane with (relative to the
% ego car 100kmph which is not that fast if you consider the ego car moving in
% the road at a speed of lets say 60kmph). And a car in the same lane in
% front of the ego car, which does not accelerate.

% *Phase code is calculate this way because #Rb*T random bits of 0 and 1s
% will be allocated to the matrix and then mulltiplying this 1D array with
% 2, gives 0s and 2s which subtracted by 1 give -1 and 1s need for the
% phase codes exponentials which give 0,π phases.

phase_code = 2 * randi([0 1], 1, Rb * T) - 1; % *Phase code for the ego car, 10kbits since 1Gbps * 10μs for the chirp period and the data rate.
target_range = [5, 30]; % Target range in meters.
target_velocity = [27.78, 13.89]; % Target velocity in m/s.
target_accelaration = [2.5, 0]; % Target accelaration in m/s^2.