function [x_new_new, P] = EKF_RSSI_SLAM(x,P,z,u,idx, R)  

    % State vector
    % x = [xr, yr, theta, x1, y1, ..., xn, yn]
    % robot position and orientation, node positions
    
    % Control vector
    % u = [delta_D, delta_Theta]
    % Linear displacement of robot, change in orientation of robot

    Q = eye(length(x))*0.1;
%     R = eye(length(z))*1000;
    
    % Prediction
    [x_new,F] = f(x,u);
    P = F*P*F' + Q;

    % Measurement Update
    [x_mm,H] = h(x_new,idx);
    y = z - x_mm';
    S = H*P*H' + R;
    K = P*H'*(S\eye(size(S)));
    x_new_new = x_new + (K*y)';
    P = (eye(size(K*H)) - K*H)*P;
end

function [x_new,F] = f(x,u,idx)
    x_new = x;
    x_new(1:3) = [x(1) + u(1)*cosd(x(3)+u(2)); ...
                  x(2) + u(1)*sind(x(3)+u(2)); ...
                  x(3) + u(2)];
    x_new(idx) = x(idx) + (sqrt((x(
    % Jacobian of f
    F = eye(length(x));
    F(1,3) = -1*u(1)*sind(x(3));
    F(2,3) = u(1)*cosd(x(3));
end

function [x_measure,H] = h(x,idx)
    lmx = x((idx-1)*2 + 4);
    lmy = x((idx-1)*2 + 5);
    % idx is the index of the observed node
    x_measure = sqrt((x(1) - lmx)^2 + (x(2) - lmy)^2);
    % Jacobian of h
    H = zeros(1,length(x));
    H(1) = (x(1) - lmx)/sqrt((lmx - x(1))^2 + (lmy - x(2))^2);
    H(2) = (x(2) - lmy)/sqrt((lmx - x(1))^2 + (lmy - x(2))^2);
    H((idx-1)*2 + 4) = -(x(1) - lmx)/sqrt((x(1) - lmx)^2 + (x(2) - lmy)^2);
    H((idx-1)*2 + 5) = -(x(2) - lmy)/sqrt((x(1) - lmx)^2 + (x(2) - lmy)^2);
end