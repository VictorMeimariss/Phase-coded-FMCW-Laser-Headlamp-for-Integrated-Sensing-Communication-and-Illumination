% 26/4/26 Victor Emmanuel Meimaris: Created decouple_signal.m script that
% takes the signal array from the previous function, generates a 2D-RDM
% which is used by the CA-CFAR [3]* algorithm extracting R, (θ is known from the 
% camera)and t is the specific frame time(R, θ, t) creating x,y,t which
% then is used to return the projected plane matrices: xy, xt and yt.
function coordinates = decouple_signal(M, N, T_chirp, signal, fs, c, B, guard_size, training_size, pfa, theta, R)
    
    % Generate RDM and Power Matrix
    RDM = fftshift(fft2(signal, N, M));
    pm = abs(RDM).^2;

    % Initialize CFAR detector with used parameters using Matlab's function
    detector = phased.CFARDetector2D('TrainingBandSize', training_size, ...
        'GuardBandSize', guard_size, 'ProbabilityFalseAlarm', pfa, ...
        'OutputFormat', 'Detection index');
    
    % Axis definitions for mapping indices of the RDM to physical values only 
    % of range, as the doppler velocity is not used in the paper and it's
    % not needed. R = fb * c * Tchirp / 2*B from fb = (B/ Tchirp) * τ

    r_axis = (linspace(-fs/2, fs/2 - fs/N, N) * c * T_chirp) / (2 * B);


    % Grid of indices calculation
    margin_r = training_size(1) + guard_size(1);
    margin_c = training_size(2) + guard_size(2);
    [rows, cols] = size(pm);
    
    idx_rows = (1 + margin_r):(rows - margin_r);
    idx_cols = (1 + margin_c):(cols - margin_c);
    
    [C_idx, R_idx] = meshgrid(idx_cols, idx_rows);
    cutidx = [R_idx(:)'; C_idx(:)'];


    % Run CFAR (dets: row 1 = Range Index, row 2 = Doppler Index)
    dets = detector(pm, cutidx);
    detected_ranges = r_axis(dets(1, :));
    
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