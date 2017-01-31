function [ node, A, n ] = generateNode( nodePos, numOfRays, sigma, type )
%generateNode Generates RSSI radiation pattern for a node
if(nargin > 3)
    [s, dist] = markovRSSI(numOfRays,type);
else
    [s, dist] = markovRSSI(numOfRays);
end

node.rays = s;
node.dist = dist;
node.pos = nodePos;
node.noise = sigma;

% Get log-distance path loss parameters
A = zeros(numOfRays,1);
n = zeros(numOfRays,1);
for ii = 1:numOfRays
    [st, n(ii), A(ii)] = fitLogDistanceModel(s(ii,:)', dist);
end

% Vector containing distances based on model
node.st = st;

% Save log-distance model parameters
node.A = mean(A);
node.n = mean(n);

A = mean(A);
n = mean(n);

