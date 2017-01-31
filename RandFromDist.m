function randval = RandFromDist(dist) 
%
% The input distribution will be of the form:
%
% dist(i) = probability(event i);
%
% Generate random sample from a provided
% discrete distribution. The returned value
% indicates the index of the selected event
% by the index of the event.
%
% i.e. if event 5 occurs we have randomly
% selected dist(5) as the event and 
% we return randval = 5.
%
% Andrew Willis
% Brown University
% February 1, 2002
%
for i=1:length(dist),
  cdf(i)=sum(dist(1:i));
end
randval = min(find(cdf > rand(1)));
