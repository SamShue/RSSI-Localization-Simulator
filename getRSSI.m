function [rssi] = getRSSI(node,robotPos)

% Get robot position relative to node position
% Get robot position in polar coords with node at origin
relativePos = [robotPos(1:2) - node.pos];
d = sqrt(relativePos(1)^2 + relativePos(2)^2);
theta = atand(relativePos(2)/relativePos(1));
while(theta < 0)
    theta = theta + 360;
end

% Get number of rays generated for each node
[numRays ~] = size(node.rays);
% Get degrees between rays
degRes = 360/numRays;

% Get nearest ray indecies
idx(1) = floor(theta/degRes)+1;
idx(2) = ceil(theta/degRes)+1;
if(idx(2) > numRays)
    idx(2) = 1;
end

% Get nearest index to distance from node
[c index] = min(abs(node.dist - d));
% if distance isn't in incremented values, use bi-linear
% interpolation
try
    [c2 index2] = min(abs(node.dist(node.dist~=node.dist(index)) - d));
    rssi(1) = (node.rays(idx(1),index)*c2 + node.rays(idx(1),index2)*c)/(c+c2);
    rssi(2) = (node.rays(idx(2),index)*c2 + node.rays(idx(2),index2)*c)/(c+c2);
    % Get rssi value by weighting according to distance to each rssi vector
    rssi = rssi(2)*(theta - floor(theta/degRes))/degRes + rssi(1)*(1 - (theta - floor(theta/degRes))/degRes);
    rssi = rssi + normrnd(0,node.noise);
catch
    idx
    % if node is in the same location as robot
    if(d == 0)
        rssi = min(node.rays(1,:));
    end
end

