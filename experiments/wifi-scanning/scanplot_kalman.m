load('scan-worxmate-guest.mat');
time = scan(:,1);
data = scan(:,2);

%% Plot kalman filter
% VECTOR VARIABLES:
%
% s.x = state vector estimate. In the input struct, this is the
%       "a priori" state estimate (prior to the addition of the
%       information from the new observation). In the output struct,
%       this is the "a posteriori" state estimate (after the new
%       measurement information is included).
% s.z = observation vector
% s.u = input control vector, optional (defaults to zero).

s.z = data;
s.x = data(1);
s.R = 0.001;
out = kalmanf(s);

% title('Low pass filter');
% xlabel('Unix timestamp');
% ylabel('RSSI');
% legend('Raw data','Filter on 10\^RSSI (\alpha = 0.1)', 'Filter on RSSI (\alpha = 0.25)');