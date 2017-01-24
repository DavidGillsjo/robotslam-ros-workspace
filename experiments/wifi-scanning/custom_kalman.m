function [ output, error_estimation ] = custom_kalman( data, error_measurement, ...
    initial_error_estimation )
%Custom kalman Inefficient kalman filter
%   Basically interpolation between previous and current value.
    output = zeros(size(data));
    output(1) = data(1);
    error_estimation = zeros(size(data));
    error_estimation(1) = initial_error_estimation;
    for i = (2:size(output))
        gain = error_estimation(i-1)/(error_estimation(i-1) + error_measurement);
        output(i) = output(i-1) + gain * (data(i) - output(i-1));
        error_estimation(i) = (1-gain) * (error_estimation(i-1));
    end
end

