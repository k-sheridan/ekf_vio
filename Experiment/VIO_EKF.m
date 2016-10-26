function VIO_EKF()

    function [Q] = getProcessCovariance(dt)
        Q = 0.03 * dt * eye(16, 16);
    end

    function [X_pred] = stateTransition(X, Z, dt)
        X_pred(1:3, 1) = X(1:3) + X(4:6)*dt + 0.5 * (quatrotate(X(7:10)', Z(4:6)')' - [0; 0; 9.8]) * dt*dt;
        X_pred(4:6, 1) = X(4:6) + (quatrotate(X(7:10)', Z(4:6)')' - [0; 0; 9.8]) * dt;
        temp = quatrotate(X(7:10)', Z(1:3)') * dt;
        X_pred(7:10, 1) = quatnormalize(quatmultiply(X(7:10)', angle2quat(temp(1), temp(2), temp(3), 'XYZ')))';
       
    end

%initialize the state vector
X = [0; 0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
%x, y, z, dx, dy, dz, qx, qy, qz, qw, bgx, bgy, bgz, bax, bay, baz
P = 0 * eye(16, 16); % make the initial values 0 uncertainty since this is odometry
P(11:16, 11:16) = 0 * eye(6, 6); % set the biases to 0 uncertianty

Z_IMU = [0; 0; pi/2; 1; 0; 1 + 9.8];

X = stateTransition(X, Z_IMU, 1)
X = stateTransition(X, Z_IMU, 1)


end