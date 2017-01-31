clc;
clear;
close all;

% Import Environment
%==========================================================================
% Environment Variables
A = 35;
n = 1.8;
len = 10;   % environment length (m)
wid = 10;   % environment width (m)
% End Import Environment
%--------------------------------------------------------------------------

% Generate Nodes
%==========================================================================
% Node Params
numOfNodes = 1;
nodeRes = 1;
for ii = 1:numOfNodes
    % Types can be Outdoors, Hallway, Lab, Large Open Room, or Outdoors
    node(ii) = generateNode([0,5],nodeRes, 3, 'Hallway');
end
% End Generate Nodes
%--------------------------------------------------------------------------

% Generate Robot
%==========================================================================
% Robot Params
robotPose = [0,5,0];    % initial robot position
robotPath = ones(length(node(1).dist),2).*[0.25,0];
% End Generate Robot
%--------------------------------------------------------------------------

% Simulation
%--------------------------------------------------------------------------
for ii = 1:length(robotPath)
    % Follow specified path
    %======================================================================
    % Rotate random amount
    oldRobotPose = robotPose;
    newRobotPose(3) = robotPose(3) + robotPath(ii,2);
    % Move forward X meters
    newRobotPose(1) = robotPose(1) + cosd(newRobotPose(3))*robotPath(ii,1);
    newRobotPose(2) = robotPose(2) + sind(newRobotPose(3))*robotPath(ii,1);    
    robotPose = newRobotPose;
    locationHistory(:,ii) = robotPose;
    % End Follow Specified Path
    %----------------------------------------------------------------------
    
    % Get odometry data
    %======================================================================
    % Calculate odometry to introduce real errors that may be encountered
    % during actual implementation
    disp = (robotPose(1:2) - oldRobotPose(1:2));
    disp = sqrt(disp(1)^2 + disp(2)^2) + normrnd(0,0.1);
    u = [disp, (robotPose(3) - oldRobotPose(3)) + normrnd(0,0.5)];
    % End odometry data
    %----------------------------------------------------------------------

    % Get RSSI Value to each node
    %======================================================================
    for jj = 1:numOfNodes
        rssi(jj) = getRSSI(node(jj),robotPose);
        dist2node(jj) = euclidDist(node(jj).pos,robotPose);
        d(jj) =  10^((rssi(jj)-A)/(10*n)); % Log-Distance Model
    end
    if(~exist('dhist'))
        dhist = d(jj);
        rhist = rssi(jj);
    else
        dhist = [dhist;d(jj)];
        rhist = [rhist;rssi(jj)];
    end
    % End Get RSSI Value to each node
    %----------------------------------------------------------------------

        
        
    % Plot Junk
    %======================================================================
    clf;
    axis([0 len 0 wid]);
    hold on;
    scatter(robotPose(1),robotPose(2),'red','o');
    for jj = 1:numOfNodes
        scatter(node(jj).pos(1),node(jj).pos(2),'blue','o');
    end
    pause(0.1);
    % End Plot Junk
    %----------------------------------------------------------------------
end
% End Simulation
%--------------------------------------------------------------------------
figure();
plot(node(1).dist,rhist);