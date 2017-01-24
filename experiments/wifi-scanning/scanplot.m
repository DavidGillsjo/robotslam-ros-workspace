load('scan-worxmate-guest-3.mat');
time = scan(:,1);
data = scan(:,2);

%% Plot no filter
%plot(time, data);

%% Plot low pass filter
alpha = 0.05;
log_y = custom_low_pass(data, alpha);
alpha = 0.25;
y = log10(custom_low_pass(10.^data, alpha));
error_measurement = 5;
error_estimation = 0.08;
y_log_kalman = custom_kalman(data, error_measurement, error_estimation);
plot(time, data, time, log_y, time, y, time, y_log_kalman);
title('Low pass filter');
xlabel('Unix timestamp');
ylabel('RSSI');
legend('Raw data','Filter on 10\^RSSI (\alpha = 0.1)', 'Filter on RSSI (\alpha = 0.25)', 'Kalman filter on RSSI');