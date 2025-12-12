% parse_and_plot_time_fft.m
% Reads lines like:
% 10:20:40.453 -> A0 Voltage: 7.3025 V, Avg: 7.2610 V
% Plots time series (measured timestamps) and single-sided FFT amplitude spectrum.

clc; clear;

filename = 'readings.txt';  % change to your file
txt = strtrim(string(splitlines(fileread(filename))));
txt(txt=="") = []; % remove blank lines

Nlines = numel(txt);
if Nlines < 2
    error('Need at least 2 samples to build a time axis and compute FFT.');
end

time_strs = strings(Nlines,1);
V = NaN(Nlines,1);
Vavg = NaN(Nlines,1);

% Parse each line
% pattern: HH:MM:SS.mmm -> A0 Voltage: 7.3025 V, Avg: 7.2610 V
pat = '^(\d{2}:\d{2}:\d{2}\.\d{1,3})\s*->\s*A0 Voltage:\s*([+-]?\d+\.?\d*)\s*V\s*,\s*Avg:\s*([+-]?\d+\.?\d*)\s*V';

for k = 1:Nlines
    ln = txt(k);
    tokens = regexp(ln, pat, 'tokens', 'once');
    if isempty(tokens)
        error('Line %d did not match expected format:\n%s', k, ln);
    end
    time_strs(k) = tokens{1};
    V(k) = str2double(tokens{2});
    Vavg(k) = str2double(tokens{3});
end

% Convert timestamp strings to datetime (today's date is irrelevant; only time-of-day used)
% Use InputFormat that accepts milliseconds (up to 3 digits)
t_dt = datetime(time_strs, 'InputFormat', 'HH:mm:ss.SSS');
% handle potential lines with only 1 or 2 fractional digits by trying a fallback:
if any(isnat(t_dt))
    t_dt = datetime(time_strs, 'InputFormat', 'HH:mm:ss.SS'); % second fallback
end
if any(isnat(t_dt))
    t_dt = datetime(time_strs, 'InputFormat', 'HH:mm:ss'); % last fallback (no ms)
end
if any(isnat(t_dt))
    error('Failed to parse some timestamps. Ensure format HH:MM:SS.mmm (milliseconds) for all lines.');
end

% Convert to seconds relative to first timestamp (numeric vector)
t_seconds = seconds(t_dt - t_dt(1));  % column vector

% Plot raw time-domain data using measured arrival times
figure('Units','normalized','Position',[0.08 0.08 0.7 0.7]);

subplot(2,1,1);
plot(t_seconds, V, '-o', 'LineWidth', 1.2, 'MarkerSize', 4);
hold on;
plot(t_seconds, Vavg, '-s', 'LineWidth', 1.0, 'MarkerSize', 4);
xlabel('Time (s) relative to first sample');
ylabel('Voltage (V)');
title('A0 Voltage and Running Average vs Measured Time');
legend('A0 Voltage','Avg','Location','best');
grid on;

% --- Prepare for FFT ---
% The timestamps may be irregular. Interpolate onto a uniform time grid using median dt.
dt_vec = diff(t_seconds);
median_dt = median(dt_vec);
if median_dt <= 0
    error('Non-increasing timestamps detected.');
end

% Create uniform time vector from first to last sample with spacing = median_dt
t_uniform = (t_seconds(1) : median_dt : t_seconds(end))';
if numel(t_uniform) < 4
    % Make sure we have enough points for FFT
    t_uniform = linspace(t_seconds(1), t_seconds(end), max(8, numel(t_seconds)))';
    median_dt = t_uniform(2)-t_uniform(1);
end

% Interpolate measured voltage onto uniform grid (use shape-preserving PCHIP)
V_uniform = interp1(t_seconds, V, t_uniform, 'pchip');
% remove NaNs if interpolation out-of-bounds (shouldn't happen)
ok = ~isnan(V_uniform);
t_uniform = t_uniform(ok);
V_uniform = V_uniform(ok);

% Sampling frequency
fs = 1/median_dt;
N = numel(V_uniform);

% Compute single-sided amplitude spectrum
Y = fft(V_uniform);
P2 = abs(Y / N);               % two-sided spectrum
% single-sided spectrum
if mod(N,2)==0
    % N even
    P1 = P2(1:N/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
else
    % N odd
    P1 = P2(1:(N+1)/2);
    P1(2:end) = 2*P1(2:end);
end
f = (0:length(P1)-1)' * (fs / N);  % frequency vector for P1

% Plot FFT (single-sided amplitude)
subplot(2,1,2);
plot(f, P1, 'LineWidth', 1.2);
xlabel('Frequency (Hz)');
ylabel('Amplitude (V)');
title(sprintf('Single-sided Amplitude Spectrum (fs = %.2f Hz, N = %d)', fs, N));
grid on;
xlim([0, min( fs/2, max(f) )]);
xscale('log')
yscale('log')

% Optional: mark Nyquist frequency
hold on;
nyq = fs/2;
ymax = max(P1);
plot([nyq nyq], [0 ymax], '--k', 'HandleVisibility','off');

% Improve layout
tightfig = get(gcf,'Position'); %#ok<NASGU>
sgtitle('Measured Voltage vs Time and Single-sided FFT');

% End of script
