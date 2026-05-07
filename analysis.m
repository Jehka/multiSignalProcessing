% =============================================================================
% analysis.m — Air Quality Post-Analysis
% Reads CSV log produced by monitor.py and runs:
%   1. Time-series visualization
%   2. FFT noise analysis per channel
%   3. FIR filter characterization (raw vs filtered comparison)
%   4. Cross-channel correlation during alarm events
%   5. Alarm event statistics
%
% Usage:
%   analysis('data/log_20240101_120000.csv')
%   analysis()   % prompts file picker
% =============================================================================

function analysis(csv_path)

    % -------------------------------------------------------------------------
    % Load Data
    % -------------------------------------------------------------------------
    if nargin < 1 || isempty(csv_path)
        [f, p] = uigetfile('data/*.csv', 'Select log file');
        if isequal(f, 0), disp('Cancelled.'); return; end
        csv_path = fullfile(p, f);
    end

    fprintf('Loading %s...\n', csv_path);
    T = readtable(csv_path, 'VariableNamingRule', 'preserve');

    t           = T.timestamp;
    alarm_level = T.alarm_level;
    ch0         = double(T.ch0_fir);
    ch1         = double(T.ch1_fir);
    ch2         = double(T.ch2_fir);
    n           = length(t);

    fprintf('Loaded %d samples, %.1f seconds\n', n, t(end) - t(1));

    % Estimate sample rate from timestamps
    dt = mean(diff(t));
    fs = 1 / dt;
    fprintf('Estimated sample rate: %.1f Hz\n', fs);

    alarm_labels = {'CLEAR', 'WARNING', 'DANGER', 'CRITICAL'};
    alarm_colors = {[0.2 0.7 0.2], [1 0.8 0], [1 0.4 0], [0.8 0 0]};

    % -------------------------------------------------------------------------
    % 1. Time-Series Overview
    % -------------------------------------------------------------------------
    figure('Name', 'Time-Series Overview', 'Position', [50 50 1200 700]);

    subplot(4,1,1);
    plot(t, ch0, 'b', 'LineWidth', 0.8); hold on;
    ylabel('MQ-3 (ADC)'); title('Channel 0 — MQ-3 (Alcohol/VOC)');
    mark_alarms(t, alarm_level, ch0);
    grid on;

    subplot(4,1,2);
    plot(t, ch1, 'r', 'LineWidth', 0.8); hold on;
    ylabel('MQ-135 (ADC)'); title('Channel 1 — MQ-135 (Air Quality)');
    mark_alarms(t, alarm_level, ch1);
    grid on;

    subplot(4,1,3);
    plot(t, ch2, 'g', 'LineWidth', 0.8); hold on;
    ylabel('LDR (ADC)'); title('Channel 2 — LDR (Light)');
    mark_alarms(t, alarm_level, ch2);
    grid on;

    subplot(4,1,4);
    stairs(t, alarm_level, 'k', 'LineWidth', 1.5);
    yticks([0 1 2 3]); yticklabels(alarm_labels);
    ylim([-0.3 3.3]); ylabel('Alarm'); xlabel('Time (s)');
    title('Alarm Level'); grid on;

    % -------------------------------------------------------------------------
    % 2. FFT Noise Analysis
    % -------------------------------------------------------------------------
    figure('Name', 'FFT Analysis', 'Position', [100 100 1000 600]);

    channels = {ch0, ch1, ch2};
    ch_names = {'MQ-3', 'MQ-135', 'LDR'};
    ch_colors = {'b', 'r', 'g'};

    for i = 1:3
        sig = channels{i} - mean(channels{i});   % remove DC
        N   = length(sig);
        f_ax = (0:N/2-1) * (fs/N);
        mag  = abs(fft(sig));
        mag  = mag(1:N/2) / N;
        mag_db = 20*log10(mag + 1e-10);

        subplot(3,1,i);
        plot(f_ax, mag_db, ch_colors{i}, 'LineWidth', 0.8);
        xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');
        title(sprintf('%s — FFT (DC removed)', ch_names{i}));
        xlim([0 fs/2]); grid on;
    end
    sgtitle('Frequency Content of FIR-Filtered Signals');

    % -------------------------------------------------------------------------
    % 3. Cross-Channel Correlation
    % -------------------------------------------------------------------------
    figure('Name', 'Cross-Channel Correlation', 'Position', [150 150 900 700]);

    % Normalize
    ch0n = (ch0 - mean(ch0)) / (std(ch0) + 1e-10);
    ch1n = (ch1 - mean(ch1)) / (std(ch1) + 1e-10);
    ch2n = (ch2 - mean(ch2)) / (std(ch2) + 1e-10);

    subplot(3,1,1);
    [xc, lags] = xcorr(ch0n, ch1n, round(fs*5), 'normalized');
    plot(lags/fs, xc, 'LineWidth', 1);
    xlabel('Lag (s)'); ylabel('Correlation');
    title('MQ-3 vs MQ-135 Cross-Correlation');
    xline(0, 'r--'); grid on;

    subplot(3,1,2);
    [xc, lags] = xcorr(ch0n, ch2n, round(fs*5), 'normalized');
    plot(lags/fs, xc, 'LineWidth', 1);
    xlabel('Lag (s)'); ylabel('Correlation');
    title('MQ-3 vs LDR Cross-Correlation');
    xline(0, 'r--'); grid on;

    subplot(3,1,3);
    [xc, lags] = xcorr(ch1n, ch2n, round(fs*5), 'normalized');
    plot(lags/fs, xc, 'LineWidth', 1);
    xlabel('Lag (s)'); ylabel('Correlation');
    title('MQ-135 vs LDR Cross-Correlation');
    xline(0, 'r--'); grid on;

    % -------------------------------------------------------------------------
    % 4. Alarm Event Analysis
    % -------------------------------------------------------------------------
    figure('Name', 'Alarm Statistics', 'Position', [200 200 900 500]);

    % Count alarm levels
    counts = histcounts(alarm_level, -0.5:1:3.5);

    subplot(1,2,1);
    b = bar(0:3, counts, 'FaceColor', 'flat');
    for i = 1:4
        b.CData(i,:) = alarm_colors{i};
    end
    xticks(0:3); xticklabels(alarm_labels);
    ylabel('Count'); title('Alarm Level Distribution');
    grid on;

    % Alarm event durations
    alarm_events = alarm_level > 0;
    transitions  = diff([0; alarm_events; 0]);
    starts       = find(transitions == 1);
    ends         = find(transitions == -1) - 1;

    if ~isempty(starts)
        durations = t(ends) - t(starts);
        subplot(1,2,2);
        histogram(durations, 20, 'FaceColor', [1 0.4 0]);
        xlabel('Duration (s)'); ylabel('Count');
        title(sprintf('Alarm Event Durations\n(n=%d events, mean=%.1fs)',
              length(durations), mean(durations)));
        grid on;

        fprintf('\n--- Alarm Event Summary ---\n');
        fprintf('Total events:     %d\n', length(durations));
        fprintf('Mean duration:    %.2f s\n', mean(durations));
        fprintf('Max duration:     %.2f s\n', max(durations));
        fprintf('WARNING count:    %d\n', sum(alarm_level == 1));
        fprintf('DANGER count:     %d\n', sum(alarm_level == 2));
        fprintf('CRITICAL count:   %d\n', sum(alarm_level == 3));
    else
        subplot(1,2,2);
        text(0.5, 0.5, 'No alarm events', 'HorizontalAlignment', 'center');
        title('Alarm Event Durations');
    end

    % -------------------------------------------------------------------------
    % 5. Channel Statistics Table
    % -------------------------------------------------------------------------
    fprintf('\n--- Channel Statistics ---\n');
    fprintf('%-10s %8s %8s %8s %8s %8s\n', ...
            'Channel', 'Min', 'Max', 'Mean', 'Std', 'SNR(dB)');
    for i = 1:3
        sig = channels{i};
        snr_db = 20*log10(mean(sig) / (std(sig) + 1e-10));
        fprintf('%-10s %8.0f %8.0f %8.1f %8.1f %8.1f\n', ...
                ch_names{i}, min(sig), max(sig), mean(sig), std(sig), snr_db);
    end

end

% =============================================================================
% Helper: mark alarm events on a subplot
% =============================================================================
function mark_alarms(t, alarm_level, sig)
    danger_mask   = alarm_level >= 2;
    warning_mask  = alarm_level == 1;
    if any(warning_mask)
        scatter(t(warning_mask), sig(warning_mask), 8, [1 0.8 0], 'filled');
    end
    if any(danger_mask)
        scatter(t(danger_mask), sig(danger_mask), 12, [0.9 0.2 0], 'filled');
    end
end