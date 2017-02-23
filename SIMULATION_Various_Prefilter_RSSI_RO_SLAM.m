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
    
    % RSSI Particle Filter
    %======================================================================
    if(strcmp(method,'Particle'))
        % Don't run particle filter until EFK has ran
        if(ii == 1)
            partDist = dist2node;
        else
            if(~exist('numParts'))
                numParts = 1000;
                for jj = 1:numParts
                    % Actual random particle values
%                     particles(jj,:) = rand([1 numOfNodes])*10 + normrnd(0,3,[1,numOfNodes]);
                    % Initialize particles with estimated distances
                    particles(jj,:) = d + normrnd(0,1,[1,numOfNodes]);
                    % Cheese it with perfect initialization
%                     particles(jj,:) = dist2node + normrnd(0,0.1,[1,numOfNodes]);
                end
                newParticles = zeros(size(particles));
            end
            % Use odometry and estimated states to update particles
            for jj = 1:numOfNodes
                % get updated state variables from measured odometry
                odom_x = [x(1) + u(1)*cosd(x(3)+u(2)),x(2) + u(1)*sind(x(3)+u(2))];
                % Extract landmark positions from state vector
                lmx = x((jj-1)*2 + 4);
                lmy = x((jj-1)*2 + 5);
                % Prediction vector contains changes to each node
                prediction(jj) = euclidDist(odom_x,[lmx,lmy]) - ...
                                 euclidDist(x(1:2),[lmx,lmy]); 
            end
            if(ii <= 2)
                p_hist = prediction;
            else
                p_hist = [p_hist;prediction];
            end
            % Apply changes to particles
            for jj = 1:numParts
                particles(jj,:) = particles(jj,:) + prediction;
            end
            % Get fitness value for each particle
            err = zeros(size(particles));
            for jj = 1:numParts
                err(jj,:) = abs(d-particles(jj,:));
            end
            for jj = 1:numParts
                err(jj,:) = abs(err(jj,:) - max(err));
            end
            for jj = 1:numParts
                err_dist(jj,:) = err(jj,:)./sum(err);
            end
            % Resample from distribution
            cdf = zeros(size(particles));
            for i=1:numParts
              cdf(i,:)=sum(err_dist(1:i,:));
            end

            for jj = 1:numParts
                for kk = 1:numOfNodes
                    randval = min(find(cdf(:,kk) > rand(1)));
                    newParticles(jj,kk) = particles(randval,kk);
                end
            end
            particles = newParticles;
            % Get most probable position
            partDist = sum(particles.*err_dist)./sum(err_dist);
            % Perturb particles
            particles = particles + normrnd(0,0.25,[numParts,numOfNodes]);
        end
        % Calculate R
        % R is calculated based on the difference between the chance in
        % estimated RSSI distance and the odometry distance.
        for jj = 1:numOfNodes
            if(ii > 1)
                % Calculate R using prediction vector from particle filter
%                 R(jj) = abs(d(jj) - prediction(jj))^2;
%                 R(jj) = ((partDist(jj)*abs(dhist(ii,jj) - dhist(ii-1,jj))));
                R(jj) = 100;
                % Calculate R using linear odom value
%                 lmx = x((jj-1)*2 + 4);
%                 lmy = x((jj-1)*2 + 5);
%                 % idx is the index of the observed node
%                 e_d2n = sqrt((x(1) - lmx)^2 + (x(2) - lmy)^2);
%                 R(jj) = abs(abs(d(jj) - e_d2n) - robotPath(ii,1))^2;
            else
                R(jj) = 1000;
            end
        end
    end
    % End RSSI Particle Filter
    %----------------------------------------------------------------------
    
    % Menegatti Filtering Method
    %======================================================================
    if (strcmp(method,'Menegatti'))  
        if(ii == 1)
            mDist = dist2node;
            mhist = [mDist];
%         else
%             for jj = 1:numOfNodes
%                 diff = d(jj) - mDist(jj);
%                 if(abs(diff) < u(1))
%                     mDist(jj) = d(jj);
%                 else
%                     if(diff > 0)
%                         mDist(jj) = mDist(jj) + u(1);
%                     else
%                         mDist(jj) = mDist(jj) - u(1);
%                     end
%                 end
%             end
%         end
        else
            for jj = 1:numOfNodes
                if((rhist(ii,jj) - rhist(ii-1,jj)) > 0)
                    % if change in rssi indicates movement away from node
                    % "Even out predicted RSSI value with measured RSSI value
                    %previous_rssi = 10.*n(jj).*log10(mDist) + A(jj);
                    expected_rssi = 10.*n(jj).*log10(mDist(jj) + u(1)) + A(jj);
                    predicted_rssi = (expected_rssi + rssi(jj))/2;
                    mDist(jj) = 10^((predicted_rssi-A(jj))/(10*n(jj))); % Log-Distance Model
                else
                    % if change in rssi indicates movement towards node
                    expected_rssi = 10.*n(jj).*log10(abs(mDist(jj) - u(1))) + A(jj);
                    predicted_rssi = (expected_rssi + rssi(jj))/2;
                    mDist(jj) = 10^((predicted_rssi-A(jj))/(10*n(jj))); % Log-Distance Model
                end
                % Calculate R
                R(jj) = ((mDist(jj)*abs(u(1) - abs(mDist(jj) - mhist(ii-1,jj)))));
                R = (1./R)*100 + 100;
            end
        end
        mhist = [mhist;mDist];
    end
    % End Menegatti Filtering Method
    %----------------------------------------------------------------------

    % Correlation Filter
    %======================================================================
    if(strcmp(method,'CorrFilt'))
        filtLen = 20;   % Sample length for filter
        if(ii == 1)
            d_hat = dist2node;
            thist = [];
            tahist = [];
            dhhist = d;
        else
            for jj = 1:numOfNodes
                if(ii - 2 > filtLen)
                    v_rssi = vectorDerivative(dhist(:,jj)); % velocity vector from estimated distances
                    r = cov(v_rssi(end-filtLen:end),vhist(end-filtLen-1:end-1,jj));
                    R(jj) = abs(r(1,2))*100000;
                    Tcc = corrcoef(v_rssi(end-filtLen:end),vhist(end-filtLen-1:end-1,jj));   % Correlation between rssi/measured velocity and odomvelocity
                else
                    v_rssi = vectorDerivative(dhist(:,jj)); % velocity vector from estimated distances
                    r = cov(v_rssi,vhist(:,jj));
                    if(ii == 1)
                        R(jj) = 1000;
                    else
                        R(jj) = abs(r(1,2))*100000;
                    end
                    Tcc = corrcoef(v_rssi,vhist(:,jj));   % Correlation between rssi/measured velocity and odomvelocity
                end
                if(length(Tcc) > 1)
                    T(jj) = (Tcc(2) + 1)/2;   % Normalize correlation coefficient
                else
                    T(jj) = Tcc;
                end
                if(isnan(T(jj)))
                    T(jj) = 0;
                end
                p_dist = d_hat(jj)+vhist(end,jj);   % Predicted distance based on change in odom
                % get new estimated distance
                d_hat(jj) = p_dist*(1-T(jj)) + d(jj)*T(jj);
            end
            thist = [thist;T];
            dhhist = [dhhist;d_hat];
        end
    end
    
    % End Correlation Filter
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
        C = 0.0;
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
        if(strcmp(method,'Particle'))
            [x,P] = EKF_RO_SLAM(x,P,partDist(jj),u,jj,100,Q);
        elseif(strcmp(method,'Menegatti'))
            [x,P] = EKF_RO_SLAM(x,P,mDist(jj),u,jj,mDist(jj)*100,Q);
        elseif(strcmp(method,'NoFilter'))
            [x,P] = EKF_RO_SLAM(x,P,d(jj),u,jj,100,Q);
        elseif(strcmp(method,'CorrFilt'))
            [x,P]= EKF_RO_SLAM(x,P,d_hat(jj),u,jj,d_hat(jj)*100,Q);
        elseif(strcmp(method,'TrustFilt'))
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

% Plot Correlation Filter Results
if(strcmp(method,'CorrFilt'))
    figure();
    v_rssi = vectorDerivative(dhist(:,1));
    subplot(2,1,1); hold on;
    plot(vhist(:,1),'red');
    plot(v_rssi(:,1),'blue');
    title('Velocities over Time');
    subplot(2,1,2);
    plot(thist(:,1),'green');
    title('Correlation Weight over Time');
    figure();
    subplot(2,1,1);
    plot(d2nhist(:,1),'blue'); hold on;
    plot(dhhist(:,1),'red');
    plot(dhist(:,1),'green');
    title('Distances over Time');
    subplot(2,1,2);
    plot(abs(d2nhist(:,1) - dhhist(:,1)),'red');
    title('Error over Time');
end

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
