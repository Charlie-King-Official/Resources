% Compare Bode plots for different K values with asymptotes
clear; clc;

% System parameters
den = conv([1 10], conv([1 1], [1 1]));  % (s+10)(s+1)^2
K_values = [0.1, 1, 10];  % Gains to compare
colors = ['r', 'g', 'b']; % Colors for plots

% Frequency range
f = logspace(-2, 2, 500);  % 0.01 to 100 rad/s

figure;
subplot(2,1,1); hold on; grid on;
title('Magnitude with Asymptotes');
xlabel('Frequency (rad/s)'); ylabel('Magnitude (dB)');

subplot(2,1,2); hold on; grid on;
title('Phase with Asymptotes');
xlabel('Frequency (rad/s)'); ylabel('Phase (deg)');

for idx = 1:length(K_values)
    K = K_values(idx);
    sys = tf(K, den);

    % Actual Bode plot data
    [mag, phase] = bode(sys, f);
    mag = 20*log10(squeeze(mag));
    phase = squeeze(phase);

    % Plot actual Bode
    subplot(2,1,1);
    semilogx(f, mag, colors(idx), 'LineWidth', 1.5);
    subplot(2,1,2);
    semilogx(f, phase, colors(idx), 'LineWidth', 1.5);

    % --- Asymptote Calculation ---
    wb1 = 1; wb2 = 10; % Break frequencies
    mag0 = 20*log10(K/(10*1^2)); % Initial magnitude
    mag_asym = zeros(size(f));
    phase_asym = zeros(size(f));

    for i = 1:length(f)
        % Magnitude asymptotes
        if f(i) < wb1
            mag_asym(i) = mag0;
        elseif f(i) < wb2
            mag_asym(i) = mag0 - 40*log10(f(i)/wb1);
        else
            mag_asym(i) = mag0 - 40*log10(wb2/wb1) - 60*log10(f(i)/wb2);
        end

        % Phase asymptotes
        if f(i) < 0.1
            phase_asym(i) = 0;
        elseif f(i) < 1
            phase_asym(i) = -90*(log10(f(i)/0.1));
        elseif f(i) < 10
            phase_asym(i) = -90 - 90*(log10(f(i)/1));
        else
            phase_asym(i) = -270;
        end
    end

    % Plot asymptotes
    subplot(2,1,1);
    semilogx(f, mag_asym, [colors(idx) '--'], 'LineWidth', 1);
    subplot(2,1,2);
    semilogx(f, phase_asym, [colors(idx) '--'], 'LineWidth', 1);
end

subplot(2,1,1);
legend('K=0.1 Actual','K=0.1 Asymptote','K=1 Actual','K=1 Asymptote','K=10 Actual','K=10 Asymptote');
subplot(2,1,2);
legend('K=0.1 Actual','K=0.1 Asymptote','K=1 Actual','K=1 Asymptote','K=10 Actual','K=10 Asymptote');
