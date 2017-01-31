function [ d ] = euclidDist( p1,p2 )
%Returns the Euclidean distance between 2 points
%   p1 is a 2x1 vector containing the first point.
%   p2 is a 2x1 vector containing the second point.
%   d is the distance between the two points.
    d = sqrt((p1(1) - p2(1))^2 + (p1(2) - p2(2))^2);
end

