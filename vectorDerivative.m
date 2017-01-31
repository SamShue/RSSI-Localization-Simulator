function [ v_dot ] = vectorDerivative( v )
%vectorDerivative takes the first derivative of a vector
%   Assuming the most recent value is at the highest index

v_dot = v(2:end) - v(1:end-1);

end

