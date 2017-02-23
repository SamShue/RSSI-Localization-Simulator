function [ rssi_out, dist_out ] = markovRSSI( numOfNodes, type )
% Get list of all files in directory
direct = 'Signal Strength Data';
listing = dir(direct);
listing = listing([listing.isdir]==1);

data = {}; distances = {}; measurements = {}; dataType = {}; name = {};

minRSSI = 1000;
maxRSSI = 0;
% Start counting at 3 to avoid using '.' and '..' as directory names
for ii = 3:length(listing)
    sublisting = dir(sprintf('%s/%s',direct,listing(ii).name));
    sublisting = sublisting([sublisting.isdir]==0);
    for jj = 1:length(sublisting)
        try
            % Open file containing distances and RSSI measurements
            data = [data;{xlsread(sprintf('%s/%s/%s',direct,listing(ii).name,sublisting(jj).name))}];
            data_tmp = cell2mat(data(end));
            distances = [distances;{data_tmp(2:41,1)}];
            measurements = [measurements;{data_tmp(2:41,2:end)}]; % Taking 1 orientation
            dataType = [dataType;{listing(ii).name}];
            name = [name;{sublisting(jj).name}];
            % record max and min values
            tmpMeasures = cell2mat(measurements(jj));
            tmpMeasures(tmpMeasures == 0) = NaN;
            tmpMax = max(max(tmpMeasures));
            tmpMin = min(min(tmpMeasures));
            if(tmpMax > maxRSSI)
                maxRSSI = tmpMax;
            end
            if(tmpMin < minRSSI)
                minRSSI = tmpMin;
            end
        end
    end
end

if(nargin > 1)
    %Select environment data type
    % type = 'Classroom';
    for ii = 1:length(dataType)
        if(strcmp(char(dataType(ii)),type))
            idx(ii) = 1;
        else
            idx(ii) = 0;
        end
    end
    if(sum(idx) == 0)
        error('Environment type not listed.');
    end
    measurements = measurements(find(idx));
end

% Create states array
dist = cell2mat(distances(1));
rssiList = minRSSI:1:maxRSSI;
% for ii = 1:length(rssiList)
%     for jj = 1:length(dist)
%         state((ii-1)*length(dist) + jj,1) = rssiList(ii);
%         state((ii-1)*length(dist) + jj,2) = dist(jj);
%     end
% end

% Train markov chain
nextState = zeros((length(rssiList)^2)*length(dist),1);  % Vector to hold probabilities to next state
% Fill next state vector with "tallies" for each observation of next state
% transitions
for ii = 1:length(measurements)
    tmpMeasures = cell2mat(measurements(ii));
    [L, W] = size(tmpMeasures);
    for jj = 1:L-1    % Observing current row (RSSI values at a particular distance)
        for kk = 1:W    % Iterate through all values (columns) in the row
            % Current RSSI position index in range of RSSI min/max values
            % Defines the current state we are looking at next state values
            % for
            currentRSSIIndex = find(rssiList == tmpMeasures(jj,kk));
            for hh = 1:W
                % Get index of next state based on RSSI/Distance
                rssiIndexVal = find(rssiList == tmpMeasures(jj+1,hh));
                % Add next state observation to next state list
                distanceOffset = (jj-1)*(length(rssiList)^2);
                rssiOffset = (currentRSSIIndex - 1)*(length(rssiList));
                nextRssiOffset = rssiIndexVal;
                nextState(distanceOffset + rssiOffset + nextRssiOffset) = nextState(distanceOffset + rssiOffset + nextRssiOffset) + 1;
%                 nextState(((jj-1)*(length(rssiList)^2) + (currentRSSIIndex-1)*length(rssiList) + rssiIndexVal)) = nextState(((currentRSSIIndex-1)*length(rssiList) + rssiIndexVal) + (jj-1)*(length(rssiList)^2)) + 1;
            end
        end
    end
end

% Normalize "tallies"
index = 1;
while(index < length(nextState))
    nextState(index:index+length(rssiList)-1) = nextState(index:index+length(rssiList)-1)./sum(nextState(index:index+length(rssiList)-1));
    index = index + length(rssiList);
end
% nextState(isnan(nextState)) = 0.001;

% Generate plots based on Markov model
seed = tmpMeasures(1,randi(length(tmpMeasures(1,:))));
for jj = 1:numOfNodes
    % Create random initial value:
    rssi(1) = seed;
    for ii = 2:length(dist)
        currentRSSIIndex = find(rssiList == rssi(ii-1));
        % find next state index
        distanceOffset = (ii-2)*(length(rssiList)^2);
        index = distanceOffset + (currentRSSIIndex - 1)*(length(rssiList)) + 1;
        distribution = nextState(index:(index+length(rssiList)-1));
        nextIndex = RandFromDist(distribution);
        if(isempty(nextIndex))
%             rssi(ii) = rssi(ii-1); %randi([min(rssiList),max(rssiList)]);
            error('No probability to next state. Error generating RSSI.');
        else
            rssi(ii) = rssiList(nextIndex);
        end
    end
    rssi_out(jj,:) = rssi;
end

dist_out = dist;
