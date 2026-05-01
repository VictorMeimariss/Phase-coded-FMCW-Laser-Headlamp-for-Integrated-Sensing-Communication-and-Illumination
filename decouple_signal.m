% 26/4/26 Victor Emmanuel Meimaris: Created decouple_signal.m script that
% takes the signal array from the previous function, generates a 2D-RDM
% which is used by the CA-CFAR [3]* algorithm extracting R, (θ is known from the 
% camera)and t is the specific frame time(R, θ, t) creating x,y,t which
% then is used to return the projected plane matrices: xy, xt and yt.
function coordinates = decouple_signal(M, N, T_chirp, signal, fs, c, B, guard_size, training_size, pfa, theta, R)
    
    % Generate RDM and Power Matrix
    RDM = fftshift(fft2(signal, N, M));
    pm = abs(RDM).^2;
    
    % Axis definitions for mapping indices of the RDM to physical values only 
    % of range, as the doppler velocity is not used in the paper and it's
    % not needed. R = fb * c * Tchirp / 2*B from fb = (B/ Tchirp) * τ
    r_axis = (linspace(-fs/2, fs/2 - fs/N, N) * c * T_chirp) / (2 * B);
    
    % Margin definitions
    margin_r = training_size(1) + guard_size(1);
    margin_c = training_size(2) + guard_size(2);

    % Fast Vectorized 2D CA-CFAR
    % Binary convolution kernel
    kernel_rows = 2 * margin_r + 1;
    kernel_cols = 2 * margin_c + 1;
    kernel = ones(kernel_rows, kernel_cols);
    
    % Hollow out the guard cells and Cell Under Test (CUT) in the center
    r_idx = (margin_r - guard_size(1) + 1) : (margin_r + guard_size(1) + 1);
    c_idx = (margin_c - guard_size(2) + 1) : (margin_c + guard_size(2) + 1);
    kernel(r_idx, c_idx) = 0;
           
    num_training_cells = sum(kernel(:));
    kernel = kernel / num_training_cells; % Normalize to calculate average noise power
    
    % Calculate CFAR threshold multiplier (alpha) for square-law detector
    alpha = num_training_cells * (pfa^(-1/num_training_cells) - 1);
    
    % Convolve to extract the background noise floor across the entire matrix
    noise_floor = conv2(pm, kernel, 'same');
    
    % Apply threshold to isolate detections
    threshold = noise_floor * alpha;
    logical_detections = pm > threshold;
    
    % Suppress edge detections
    logical_detections(1:margin_r, :) = false;
    logical_detections((end-margin_r+1):end, :) = false;
    logical_detections(:, 1:margin_c) = false;
    logical_detections(:, (end-margin_c+1):end) = false;
    
    % Extract Row (Range) indices of valid detections
    [det_range_idx, ~] = find(logical_detections);
    detected_ranges = r_axis(det_range_idx);
    % ----------------------------------
    
    % Sorting the values to use them on the algorithm
    ranges = sort(detected_ranges(:)); 
    ranges = ranges(ranges >= 0); % Keep only positive ranges
    
    if ~isempty(ranges)
        % dbscan(data, epsilon, minpts). Noise points return as -1.
        % The resolution is also set to 0.2 as it would mean its the same
        % car, since it will have same speed, range and theta practically
        cluster_idx = dbscan(ranges, 0.2, 3);
        
        num_clusters = max(cluster_idx);
        cluster_means = zeros(1, num_clusters);
        
        for i = 1:num_clusters
            cluster_means(i) = mean(ranges(cluster_idx == i));
        end
    else
        cluster_means = [];
    end
    
    % Assign theta to each clustered range
    assigned_thetas = zeros(1, length(cluster_means));
    tolerance = 0.038; % Distance threshold in meters
    
    for k = 1:length(cluster_means)
        [min_diff, linear_idx] = min(abs(cluster_means(k) - R(:)));
        
        if min_diff <= tolerance
            assigned_thetas(k) = theta(linear_idx);
        else
            assigned_thetas(k) = (rand() * 360) - 180; 
        end
    end
    
    % Calculate the physical coordinates based on the range from the cluster
    % means and theta
    x = cluster_means .* cosd(assigned_thetas);
    y = cluster_means .* sind(assigned_thetas);
    coordinates = [x' y'];
end