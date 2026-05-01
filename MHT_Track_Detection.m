% This is the actual implementation of the Multidimensional Hough Transform.
% Implementation details can be found on section III of the paper.

function tracks = MHT_Track_Detection(point_cloud, varargin)
    p = inputParser;
    
    % point_cloud contains the simulated data [x, y, t]
    % sorting of the input data based on time is not needed, as long as
    % future data doesn't arrive before present data, which is always the case
    
    addParameter(p, 'tracks', {}); % already existing tracks
    addParameter(p, 'window_length', 2); % rolling window length in seconds
    addParameter(p, 'gridres', 0.5); % grid resolution for converting points to binary images
    addParameter(p, 'tau', 2.0); % distance threshold for grouping points to a track
    addParameter(p, 'stitch_threshold', 5.0); % cost threshold for track stitching
    addParameter(p, 'minimum_common_points', 10); % used for AND-Logic Fusion
    addParameter(p, 'pos_weight', 1); % positional weight
    addParameter(p, 'kin_weight', 0.5); % kinematic weight
    
    addParameter(p, 'num_of_peaks', 10.0);
    % each peak corresponds to a line, so this is the total number of lines
    % the HT will detect in each window (the threshold is zero so it will
    % always detect this many)
    % you might be tempted to lower this at the minimum expected number of
    % your targets, however the accumulation points of some targets are
    % stronger than others (see debug_hough.m on how to inspect the transform)
    % and since houghpeaks selects the highest valued peaks first, your
    % remaining targets might be missed all together, so always set this
    % higher than your expected targets by a margin
    
    % the only problem is that the intersection of the project plane later
    % on has O(n^3) complexity, so a larger number of peaks make this
    % program significantly slower (in this matlab implementation at least)
    % the only way to avoid this would be to make a custom houghpeaks that
    % only detects one peak per local area, however closely spaced parallel
    % tracks, which are the most important for safety reasons (like a car
    % overtaking the ego vehicle), have very close rho and theta values so
    % a houghpeaks implementation of that nature wouldn't be feasible
                                         
    addParameter(p, 'gap', 1);
    % maximum distance a car can travel between two frames,
    % needed to avoid trajectories being intertwined (you can comment out
    % the gap check and see what happens at the unit tests for fun if you
    % want to)
    % make sure to set this based on your frame rate
    
    addParameter(p, 'houghpeaks_thres_coeff', 0);
    % peak threshold for the HT, essentially saying to our system "below
    % this threshold these don't count as line, ignore them", however we
    % always want our algorithm to detect the specified num_of_peaks, so
    % there's no point in setting a threshold, you can however if you want
    % to
    
    parse(p,varargin{:});
    
    window_length = p.Results.window_length;
    gridres = p.Results.gridres;
    tau = p.Results.tau;
    stitch_threshold = p.Results.stitch_threshold;
    minimum_common_points = p.Results.minimum_common_points;
    pos_weight = p.Results.pos_weight;
    kin_weight = p.Results.kin_weight;
    num_of_peaks = p.Results.num_of_peaks;
    gap = p.Results.gap;
    houghpeaks_thres_coeff = p.Results.houghpeaks_thres_coeff;
    
    total_tracks = length(p.Results.tracks);
    tracks = [p.Results.tracks, cell(1, 100-total_tracks)];
  
    % rolling window processing
    t_start = 0;
    while t_start < max(point_cloud(:,3))
        t_end = t_start + window_length;
        
        % extract only the points within the current window
        window_indices = t_start <= point_cloud(:,3) & point_cloud(:,3) < t_end;
        window_points = point_cloud(window_indices, :);
        
        if size(window_points, 1) > minimum_common_points
            x = window_points(:,1);
            y = window_points(:,2);
            t = window_points(:,3);
            
            % each line in a set is represent by a two column matrix of its points
            lines_set_xy = process_plane(x, y, gridres, tau, num_of_peaks, houghpeaks_thres_coeff);
            lines_set_xt = process_plane(x, t, gridres, tau, num_of_peaks, houghpeaks_thres_coeff);
            lines_set_yt = process_plane(y, t, gridres, tau, num_of_peaks, houghpeaks_thres_coeff);
            
            valid_segments = and_logic_fusion( ...
                window_points, ...
                lines_set_xy, ...
                lines_set_xt, ...
                lines_set_yt, ...
                gap, ...
                minimum_common_points ...
            );
            
            [tracks, total_tracks] = stitch_tracks( ...
                tracks, ...
                total_tracks, ...
                valid_segments, ...
                pos_weight, ...
                kin_weight, ...
                stitch_threshold ...
            );
        end
        
        t_start = t_start + (window_length / 2); % overlapping rolling window
    end
    
    tracks = tracks(1:total_tracks);  % trimming empty space
    
    for index = 1:total_tracks
        % keep only unique points, due to the overlapping window some points are the same
        tracks{index}.points = unique(tracks{index}.points, 'rows', 'stable');
    end
end


function lines_set = process_plane(dim1, dim2, gridres, tau, num_of_peaks, houghpeaks_thres_coeff)
    % convert from a cartesian plane to a Black & White binary image
    min1 = min(dim1); max1 = max(dim1);
    min2 = min(dim2); max2 = max(dim2);
    
    num_cols = round((max1 - min1) / gridres) + 1;
    num_rows = round((max2 - min2) / gridres) + 1;
    
    BW = false(num_rows, num_cols);
    
    col_idx = round((dim1 - min1) / gridres) + 1;
    row_idx = round((dim2 - min2) / gridres) + 1;
    
    linear_indices = sub2ind(size(BW), row_idx, col_idx);
    BW(linear_indices) = true;
    
    % perform 2D Hough
    [H, theta, rho] = hough(BW, 'RhoResolution', 1, 'Theta', -90:1:89.5);
    
    % apply a 3x3 mean filter
    mean_filter = fspecial('average', [3 3]);
    H = imfilter(H, mean_filter);
    H = round(H); % houghpeaks requires integers
    
    % each peak corresponds to a unique line: rho = x*cos(theta) + y*sin(theta)
    peaks = houghpeaks(H, num_of_peaks, 'threshold', ceil(houghpeaks_thres_coeff * max(H(:))));
    
    % for each line find which original points are close
    % enough to it and group them together
    theta_rad = deg2rad(theta(peaks(:,2)));
    rho_val = rho(peaks(:,1));

    dist = abs(col_idx * cos(theta_rad) + row_idx * sin(theta_rad) - rho_val);

    lines_set = cell(size(peaks,1), 1);
    for idx = 1:size(peaks,1)
        lines_set{idx} = find(dist(:, idx) < tau);
    end
end


function valid_segments = and_logic_fusion(window_points, L_xy, L_xt, L_yt, gap, minimum_common_points)
    N_xy = length(L_xy);
    N_xt = length(L_xt);
    N_yt = length(L_yt);
    
    valid_segments = cell(1, N_xy * N_xt * N_yt); 
    counter = 0;
    
    % the code below tries to escape the full O(n^3) loop as much as possible
    
    L_xy_lengths = cellfun(@length, L_xy); % cache lengths
    
    for l = 1:N_xy
        if L_xy_lengths(l) < minimum_common_points; continue; end
        A = L_xy{l};
        
        for m = 1:N_xt
            B = L_xt{m};
            
            common_AB = A(ismember(A, B)); % fast intersection using ismember, order doesn't break
            if length(common_AB) < minimum_common_points, continue; end
            
            for p = 1:N_yt
                C = L_yt{p};
                
                common_indices = common_AB(ismember(common_AB, C)); % final intersection
                
                if length(common_indices) < minimum_common_points; continue; end
                
                segment_points = window_points(common_indices, :);
                
                xy = segment_points(:, 1:2);
                
                differences = xy(2:end, :) - xy(1:end-1, :);
                
                if max(sum(differences.^2, 2)) > gap^2; continue; end

                % the below calculations are needed for stitch_tracks()
                time = segment_points(:,3);
                centroid = mean(xy);
                mean_time = mean(time);

                time = segment_points(:, 3);
                time_span = time(end) - time(1);
                
                if time_span > 0
                    vel = (xy(end,:) - xy(1,:)) / time_span;
                else
                    vel = [0, 0]; % if time is a single snapshot
                end

                counter = counter + 1;
                valid_segments{counter} = struct(...
                    'points', segment_points, ...
                    'centroid', centroid, ...
                    'mean_time', mean_time, ...
                    'velocity', vel ...
                );
            end
        end
    end
    
    valid_segments = valid_segments(1:counter); 
end

function [tracks, total_tracks] = stitch_tracks(tracks, total_tracks, segments, pos_weight, kin_weight, stitch_threshold)
    
    has_been_stitched = false(1, 100); % to avoid stitching to the same track twice

    for l = 1:length(segments)
        segment = segments{l};
        best_match_index = -1;
        lowest_cost = inf;
        
        % calculate the cost of all established tracks
        for m = 1:total_tracks
            track = tracks{m};
            
            time_gap = segment.mean_time - track.mean_time;
            predicted_pos = track.centroid + (track.velocity * time_gap);
            D_pos = norm(segment.centroid - predicted_pos);
            D_kin = norm(segment.velocity - track.velocity);
            
            cost = pos_weight * D_pos + kin_weight * D_kin;
            
            if cost < lowest_cost
                lowest_cost = cost;
                best_match_index = m;
            end
        end
        
        if lowest_cost < stitch_threshold
            if has_been_stitched(best_match_index) == false
                tracks{best_match_index}.points = [tracks{best_match_index}.points; segment.points];
                tracks{best_match_index}.centroid = segment.centroid;
                tracks{best_match_index}.mean_time = segment.mean_time;
                tracks{best_match_index}.velocity = segment.velocity;

                has_been_stitched(best_match_index) = true;
            else
                continue; % move on to the next segment
            end
        else
            total_tracks = total_tracks + 1;
            tracks{total_tracks} = segment;
            has_been_stitched(total_tracks) = true;
        end
    end
end