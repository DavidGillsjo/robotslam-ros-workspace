function [ output ] = custom_low_pass( data, alpha )
%Custom Low Pass Inefficient low pass filter
%   Basically interpolation between previous and current value.
    output = zeros(size(data));
    output(1) = data(1);
    for i = (2:size(output))
        output(i) = output(i-1) + alpha * (data(i) - output(i-1));
    end

end

