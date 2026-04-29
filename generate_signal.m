% 14/4/26 Victor Emmanuel Meimaris: Created generate_signal.m script that generates
% a signal being sent from the laser and coming back, getting mixed with
% the original and filtered.

function signal = generate_signal(t_chirp, M, m_slope, T_chirp, Rb, fs, N, fc, SNR, num_interf, fD, tau)

    % For the mix beat_signal = sRx *conj(sLo), that would remove the terms
    % the sLO has from the sRX which from (t_chirp - tau)^2 has pi*m_slope* t_chirp^2
    % and it also has 2pi*fc*t_chirp. That removes the need for a
    % computation for the sLO and we just calculate the sIF which is the
    % beat signal

    beat_signal = zeros(size(t_chirp, 2), M); % Mixed signal sRX * sLO, initiated by zeros

    % *[1] Phase code matrix for the ego car, N = 10kbits since 1Gbps * 10μs for each 256 chirps.
    % The phase code as stated in the paper is a continous stream, so we
    % generate a matrix of known phase codes for each chirp which we will
    % use again to identify
    phase_code = pi * randi([0 1], Rb * T_chirp, M); % We use random 0s and 1s, multiplying by pi later gives phase pi or 0.
    
    % Loop to generate 256 Chirps
    for i = 0: M - 1
        t_frame = 0;
        t_frame = t_frame + i * T_chirp; % Frame clock. Using it we produce 256 chirp beat signals
        % Loop for each vehichle 
        for j = 1:size(tau, 2)
            % based on the paper formula we can see that fbeat = m_slope * tau
            fb = m_slope * tau(j);
    
            %Calculate sample delay, as t_chirp< τ, φ(t - τ) = 0
            n_tau = round(tau(j) * fs); 
                
            %Padding logic: Bits only arrive after n_tau
            delayed_phase = zeros(N, 1);
            if n_tau < N
                % Map bits to samples. Since fs = 2*Rb, each bit covers 2 samples.
                % bit_idx maps the sample index back to the bit index
                sample_indices = (n_tau + 1 : N);
                bit_indices = ceil((sample_indices - n_tau) * (Rb/fs));
                delayed_phase(sample_indices) = phase_code(bit_indices, i + 1);
            end
            % Phase term from the paper but I have inverted the sign(this
            % changes nothing since the range is absolute, it moves the
            % range to positive frequencies for our ease).
            phase_term = 2 * pi * fb *t_chirp' + delayed_phase + 2 * pi * fD(j) * t_frame -2 * pi * fc * tau(j);

            % Adding everything to the beat signal
            beat_signal(:, i + 1) = beat_signal(:, i + 1) + exp(1i * phase_term); 
        end
    end

    % Adding realistic interference to the beat signal from other cars,
    % This was taken from gemini, will research another time, I want to
    % test
    for k = 1:num_interf
        % Random parameters for an uncoordinated car
        m_inter = m_slope * (1 + 0.05 * (rand - 0.5)); % 5% slope mismatch
        t_start_rand = rand * T_chirp; % Random start time
        
        % The interference beat frequency sweeps: f_int(t) = Delta_mu * t + offset
        % We model the mixed product directly:
        interf_chirp = exp(1i * (pi * (m_inter - m_slope) * t_chirp'.^2 + 2 * pi * rand * fs * t_chirp' * t_start_rand + rand * 2 * pi));
        
        % Interference power is usually lower due to path loss but I'll put
        % 1.0 just in case
        beat_signal = beat_signal + 1.0 * interf_chirp; 
    end

    % Adding noise to the beat_signal matrix
    beat_signal = awgn(beat_signal, SNR, 'measured');
    

    %!!!!!!!!!!!!!!!!!!!

    % All this math is presented in reference [8] pages 3 and 4

    %!!!!!!!!!!!!!!!!!!!

    % Once the beat_signal matrix is generated, a Group Delay Filter needs to be
    % applied. This cancels the phase coding, restoring the linear frequency
    % modulation (LFM) structure and improving range-Doppler estimation accuracy
    
    % Moving the signal to the frequency domain with fft
    signal = fft(beat_signal, [], 1);
    % FFT ranges from 0 to fs, meaning that to apply the gdf filter which
    % spans from -fs/2 to fs/2 we have to shift so..
    signal = fftshift(signal, 1);

    % Creating gdf
    f = linspace(-fs/2, fs/2 - fs/N, N)';
    gdf = exp(1i * (pi / m_slope) .* f.^2);

    % Applying filter
    signal = signal .* gdf;

    % Inversing back to time domain
    signal = ifftshift(signal, 1);
    signal= ifft(signal, [], 1);

    % Due to applying this filter, dispersion is caused and needs to be
    % dealt with
    
    for i = 1:M
        % Create the baseband reference signal
        resampled_code = repelem(phase_code(:, i), fs/Rb); % Stretches the phase code to the number of samples for fs
        ref_signal = exp(1i * resampled_code(1:N)); % Turning 1s and 0s to pi and 0 phase
        
        % Apply GDF identical dispersion to the reference
        ref_signal = fftshift(fft(ref_signal, [], 1), 1);
        ref_signal = ref_signal .* gdf;
        dispersed_ref = ifft(ifftshift(ref_signal, 1), [], 1); % Returning back to the time domain
        
        % Decode by multiplying with the conjugate of the dispersed reference
        signal(:, i) = signal(:, i) .* conj(dispersed_ref);
    end
    % Now that the signal is decoded, in the main it needs to get passed to
    % the next function for 2D-FFT/CFAR
end