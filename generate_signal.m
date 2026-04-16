function signal = generate_signal(t_chirp, M, m_slope, T_chirp, Rb, fc, SNR, num_interf, fD, tau)
    
    % For the mix beat_signal = sRx *conj(sLo), that would remove the terms
    % the sLO has from the sRX which from (t_chirp - tau)^2 has pi*m_slope* t_chirp^2
    % and it also has 2pi*fc*t_chirp. That removes the need for a
    % computation for the sLO and we just calculate the sIF which is the
    % beat signal

    beat_signal = zeros(size(t_chirp, 2), M); % Mixed signal sRX * sLO, initiated by zeros
    
    % *[1] Phase code matrix for the ego car, 10kbits since 1Gbps * 10μs for each 256 chirps.
    % The phase code as stated in the paper is a continous stream, so we
    % generate a matrix of known phase codes for each chirp which we will
    % use again to identify 

    phase_code = randi([0 1], Rb * T_chirp, M); % We use random 0s and 1s, multiplying by pi later gives phase pi or 0.
    for i = 0: M - 1
        t_frame = 0;
        t_frame = t_frame + i * T_chirp; % Frame clock. Using it we produce 256 chirp beat signals
        for j = 1:size(tau, 2)
            beat_signal(:, i + 1) = beat_signal(:, i + 1) + exp(1i *(-2 * pi * fc * tau(j) + pi * m_slope * (- 2 .* t_chirp' .* tau(j) + tau(j).^2) + pi * phase_code(:, i+1) + 2 * pi * fD(j) .* (t_frame + t_chirp')));
            
            % based on this formula we can see that fbeat = m_slope * tau,
            % since Rb = 1Gbps it's way bigger than twice the fbeat for a range
            % of up to 375m which is way more than needed, so fs = Rb. The
            % previous calculation is done using this formula:
            %fs>= 2*f = 2 * μ * 2 * R / c => for fs = Rb, Rmax < 375meters
        end
    end

    % Adding interference to the beat signal
    interf_phase_code = randi([0 1], size(beat_signal)) * pi;
    interf_s = sqrt(num_interf) .* exp(1i * interf_phase_code);

    beat_signal = beat_signal + interf_s;

    % Adding noise to the beat_signal matrix
    beat_signal = awgn(beat_signal, SNR, 'measured');

    % Once the beat_signal matrix is generated, a Group Delay Filter needs to be
    % applied. This cancels the phase coding, restoring the linear frequency
    % modulation (LFM) structure and improving range-Doppler estimation accuracy
    signal = 1;% goodnight
end