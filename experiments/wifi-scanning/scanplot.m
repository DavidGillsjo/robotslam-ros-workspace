load('scan-worxmate-guest.mat');
time = scan(:,1);
data = scan(:,2);

%% Plot no filter
%plot(time, data);

%% Plot low pass filter
alpha = 0.05;
log_y = custom_low_pass(data, alpha);
alpha = 0.25;
y = log10(custom_low_pass(10.^data, alpha));
plot(time, data, time, log_y, time, y);
title('Low pass filter');
xlabel('Unix timestamp');
ylabel('RSSI');
legend('Raw data','Filter on 10\^RSSI (\alpha = 0.1)', 'Filter on RSSI (\alpha = 0.25)');