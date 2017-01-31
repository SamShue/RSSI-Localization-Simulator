clc;
clear;
close all;

% Import Environment
%==========================================================================
% Environment Variables
A = 40;
n = 1.6;
len = 5;   % environment length (m)
wid = 5;   % environment width (m)
% End Import Environment
%--------------------------------------------------------------------------

% Generate Nodes
%==========================================================================
% Node Params
numOfNodes = 4;
nodeRes = 6;
for ii = 1:numOfNodes
    % Types can be Outdoors, Hallway, Lab, Large Open Room, or Outdoors
    [node(ii),A(ii),n(ii)] = generateNode([rand*len,rand*wid],nodeRes, 3, 'Outdoors');
end
% End Generate Nodes
%--------------------------------------------------------------------------

% Generate Robot
%==========================================================================
% Robot Params
robotPose = [1,1,0];    % initial robot position
squarePath = [0.5,0;0.5,0;0.5,0;0.5,0;0.5,0;0.5,0;0.5,0;0.5,0; ...
             0,30;0,30;0,30;0.5,0;0.5,0;0.5,0;0.5,0;0.5,0;0.5,0;0.5,0;0.5,0; ...
             0,30;0,30;0,30;0.5,0;0.5,0;0.5,0;0.5,0;0.5,0;0.5,0;0.5,0;0.5,0; ...
             0,30;0,30;0,30;0.5,0;0.5,0;0.5,0;0.5,0;0.5,0;0.5,0;0.5,0;0.5,0; ...
             0,30;0,30;0,30];
squarePath(:,1) = 0.25;
robotPath = [squarePath;squarePath;squarePath];
errPer = 0.30;  % odometry error percentage
% End Generate Robot
%--------------------------------------------------------------------------

% Select RSSI Filtering Method
%==========================================================================
% Method can be Menegatti,Particle,CorrFilt,NoFilter or Test (Test is default case)
method = 'TrustFilt';
% End Select RSSI Filtering Method
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
    disp = sqrt(disp(1)^2 + disp(2)^2) + normrnd(0,robotPath(ii,1)*errPer);
    u = [disp, (robotPose(3) - oldRobotPose(3)) + normrnd(0,1)];
    
    % Get dead reckoning position
    if(ii == 1)
        dr_pose = oldRobotPose + [u(1)*cosd(oldRobotPose(3)+u(2)) u(1)*sind(oldRobotPose(3)+u(2)) u(2)];
        ohist = u(1);   % odometry history;
        % Get dead reckoned distance to each node
        for jj = 1:numOfNodes
            temp(jj) = euclidDist(dr_pose(ii,1:2),node(jj).pos);
        end
        o2nhist = temp;
    else
        dr_pose = [dr_pose; (dr_pose(ii-1,1) + u(1)*cosd(dr_pose(ii-1,3)+u(2))), ...
                            (dr_pose(ii-1,2) + u(1)*sind(dr_pose(ii-1,3)+u(2))), ...
                            (dr_pose(ii-1,3) + u(2))];
        ohist = [ohist;u(1)];
        % Get dead reckoned distance to each node
        for jj = 1:numOfNodes
            temp(jj) = euclidDist(dr_pose(ii,1:2),node(jj).pos);
        end
        o2nhist = [o2nhist;temp];
    end
%     u = [robotPath(ii,1),robotPath(ii,2)];
    % End odometry data
    %----------------------------------------------------------------------

    % Get RSSI Value and distance estimate to each node
    %======================================================================
    for jj = 1:numOfNodes
        rssi(jj) = getRSSI(node(jj),robotPose);
        dist2node(jj) = euclidDist(node(jj).pos,robotPose);
        d(jj) =  10^((rssi(jj)-A(jj))/(10*n(jj))); % Log-Distance Model
        if(dist2node(jj) > max(node(jj).dist))
            warning('Distance to node exceeds maximum value.');
        end
    end 
   
    % Record history of rssi and distance values
    if(ii == 1)
        dhist = d;  % estimated distance history
        rhist = rssi;   % measured rssi history
        d2nhist = dist2node;
    else
        dhist = [dhist;d];
        rhist = [rhist;rssi];
        d2nhist = [d2nhist;dist2node];
    end
    % End Get RSSI Value and distance estimate to each node
    %----------------------------------------------------------------------
    
    % Get change in distance relative to each node
    %======================================================================
    if(ii == 2)  % Run filter after 2 measurements are made to integrate
            % Also to match vector lenght of rhist after differentiation
        for jj = 1:numOfNodes
            d1 = euclidDist([x((jj-1)*2 + 4),x((jj-1)*2 + 5)],x(1:2));
            % get updated state variables from measured odometry
            odom_x = [x(1) + u(1)*cosd(x(3)+u(2)),x(2) + u(1)*sind(x(3)+u(2))];
            d2 = euclidDist([x((jj-1)*2 + 4),x((jj-1)*2 + 5)],odom_x(1:2));
            v_d2n(jj) = d2 - d1;
        end
        vhist = v_d2n;
    elseif(ii > 2)
        for jj = 1:numOfNodes
            d1 = euclidDist([x((jj-1)*2 + 4),x((jj-1)*2 + 5)],x(1:2));
            % get updated state variables from measured odometry
            odom_x = [x(1) + u(1)*cosd(x(3)+u(2)),x(2) + u(1)*sind(x(3)+u(2))];
            d2 = euclidDist([x((jj-1)*2 + 4),x((jj-1)*2 + 5)],odom_x(1:2));
            v_d2n(jj) = d2 - d1;
        end
        vhist = [vhist;v_d2n];
    end
    % End get change in distance relative to each node
    %----------------------------------------------------------------------
    
    % Trust Filter
    %======================================================================
    if(strcmp(method,'TrustFilt'))
        filtLen = 5;   % Sample length for filter
        if(ii == 1)
            d_hat = dist2node;
            thist = [];
            tahist = [];
            dhhist = d_hat;
        else
            for jj = 1:numOfNodes
                % Calculated covariance and velocities for current node
                if(ii - 2 > filtLen)
                    v_rssi = vectorDerivative(dhist(:,jj)); % velocity vector from estimated distances
                    r = cov(v_rssi(end-filtLen:end),vhist(end-filtLen-1:end-1,jj));
                else
                    v_rssi = vectorDerivative(dhist(:,jj)); % velocity vector from estimated distances
                    r = cov(v_rssi,vhist(:,jj));
                end

                p_dist = d_hat(jj)+vhist(end,jj);   % Predicted distance based on change in odom
%                 T(jj) = 1/(1+abs(vhist(end,jj) - v_rssi(end))^2);
                x0 = 0.125;
                k = -log(0.5)/x0;
                T(jj) = exp(-abs(vhist(end,jj) - v_rssi(end))*k);%*(1/(1+(d(jj)^2)));
                if(ii - 2 > filtLen)
                    avg = tsmovavg([thist(end-filtLen+1:end,jj);T(jj)],'e',filtLen,1);
                    T_avg(jj) = avg(end);
                    d_hat(jj) = p_dist*(1-T_avg(jj)) + d(jj)*T_avg(jj);
                else
                    T_avg(jj) = T(jj);
                    d_hat(jj) = p_dist*(1-T(jj)) + d(jj)*T_avg(jj);
                end
            end
            thist = [thist;T];
            tahist = [tahist;T_avg];
            dhhist = [dhhist;d_hat];
        end
    end
    % End Trust Filter
    %----------------------------------------------------------------------
    
    % RSSI EKF RO-SLAM
    %======================================================================
    % Initialize EKF Variables
    if(ii == 1)
        x = [oldRobotPose];
        for jj = 1:numOfNodes
            x((jj-1)*2 + 4) = node(jj).pos(1) + normrnd(0,0.1);
            x((jj-1)*2 + 5) = node(jj).pos(2) + normrnd(0,0.1);
            initNodePos(jj,:) = [x((jj-1)*2 + 4),x((jj-1)*2 + 5)];
        end
        % Covariance Matrix
        P = eye(length(x)).*0.1; % Initializing with near-exact values
        P(1,1) = 0.1; P(2,2) = 0.1; P(3,3) = 0.1;
        % Measurement Noise
        R = ones(numOfNodes,1)*100; % Set initial uncertainty (noise) high
        % Process Noise
        C = 0.1;
        W = [u(1)*cosd(x(3)) u(1)*sind(x(3)) u(2)]';
        Q = zeros(size(P));
        Q(1:3,1:3) = W*C*W';
    end
    
    % Apply EKF for each landmark/node observation
    for jj = 1:numOfNodes
        if(jj > 1)
            u = [0, 0];
        end
        % Apply Range-Only EKF SLAM
        % Calculate Process Noise
        C = 0.1;
        W = [u(1)*cosd(x(3)) u(1)*sind(x(3)) u(2)]';
        Q = zeros(size(P));
        Q(1:3,1:3) = W*C*W';
        % Apply filter
        if(strcmp(method,'TrustFilt'))
            [x,P]= EKF_RO_SLAM(x,P,d_hat(jj),u,jj,d_hat(jj)*100,Q);
        else
            [x,P] = EKF_RO_SLAM(x,P,dist2node(jj),u,jj,100,Q); % Test EKF
        end
    end
    % record estimated position history
    if(~exist('xhist'))
        xhist = x(1:2);
    else
        xhist = [xhist;x(1:2)];
    end
    % End EKF SLAM
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

    % Plot EKF SLAM Values
    scatter(x(1),x(2),'black','x');
    lms = reshape(x(4:end),[2,numOfNodes])';
    scatter(lms(:,1),lms(:,2),'magenta','x');
    % Plot dead reckoned position
    scatter(dr_pose(ii,1),dr_pose(ii,2),'yellow','x');
    pause(0.1);
    % End Plot Junk
    %----------------------------------------------------------------------
end
% End Simulation
%--------------------------------------------------------------------------

% Plot Results
%==========================================================================
figure();
hold on;
% estimated position path
plot(xhist(:,1),xhist(:,2),'red');
% true position path
plot(locationHistory(1,:),locationHistory(2,:),'black');
% dead reckoning path
plot(dr_pose(:,1),dr_pose(:,2),'yellow');
for jj = 1:numOfNodes
    scatter(node(jj).pos(1),node(jj).pos(2),'blue','o');
    scatter(initNodePos(jj,1),initNodePos(jj,2),'magenta','o');
    plot([lms(jj,1),initNodePos(jj,1)],[lms(jj,2),initNodePos(jj,2)],'magenta');
end
scatter(lms(:,1),lms(:,2),'magenta','x');
scatter(x(1),x(2),'black','x');

% % Plot Menegatti Distances for node 1
% figure();
% hold on;
% plot(mhist(:,1),'red');
% plot(dhist(:,1),'blue');
% title('Estimated and True Distances to Node 1');

% Plot Error over time
for ii = 1:length(robotPath)
    err(ii) = (euclidDist(xhist(ii,:),locationHistory(1:2,ii)'));
end
figure();
plot(err,'red');
title('Localization Error over Iterations');
xlabel('Iteration');
ylabel('Error (m)');

% Plot Trust Filter Results
if(strcmp(method,'TrustFilt'))
    for jj = 1:numOfNodes
        figure();
        subplot(4,1,1); hold on;
        plot(dhhist(:,jj),'red');
        plot(dhist(:,jj),'green');
        plot(d2nhist(:,jj),'blue');
        plot(o2nhist(:,jj),'yellow');
        legend('filtered','unfiltered','actual','dead-reckoned');
        title(strcat('Distance to Node over Time: Node ',num2str(jj)));
        v_rssi = vectorDerivative(dhist(:,jj));
        subplot(4,1,2); hold on;
        plot(vhist(:,jj),'red');
        plot(v_rssi,'blue');
        title(strcat('Velocities over Time: Node ',num2str(jj)));
        legend('odom','rssi');
        subplot(4,1,3);
        plot(thist(:,jj),'green'); hold on;
        plot(tahist(:,jj),'blue');
        title(strcat('Trust Weight over Time: Node ',num2str(jj)))
        legend('unaveraged','averaged');
        subplot(4,1,4);
        plot(abs(d2nhist(:,jj) - dhhist(:,jj)),'red');
        title(strcat('Error over Time: Node ',num2str(jj)));
    end
end
