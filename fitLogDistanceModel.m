function [ modelvals, n, A ] = fitLogDistanceModel( rssi, dist )
% Fits RSSI - distance values to log-distance path loss model
%   Detailed explanation goes here
tbl = table(dist, rssi, 'VariableNames',{'distances','rssi'});
beta0 = [1, 5];
modelfun = @(b,dist)10.*b(1).*log10(dist) + b(2);
mdl = fitnlm(tbl,modelfun,beta0);
b = mdl.Coefficients{1:2,{'Estimate'}};
n = b(1);
A = b(2);

% x = 0.25:0.25:10;
x = dist;
modelvals = 10.*n.*log10(x) + A;
end

