%standard deviation of measurements
R_x = 0.0045;
R_v = 0.0011;

%generate random process data with given noise

%           x v 500x2
x = repmat([8 0],500,1);


%generate random measuremet data with given noise

%   x v 500x2
z = [x(:,1) + randn(500,1)*R_x x(:,2) + randn(500,1)*R_v];

baseline_x_rmse = sqrt(sum((x(:,1) - z(:,1)).^2)/500);
baseline_xv_rmse = sqrt(sum((x(:,2) - z(:,2)).^2)/500);

% Sensor noise
%   2x2
R = [R_x^2 0; 
     0 R_v^2];

% Proccess noise
Q = zeros(2);

% State transfer function
H = eye(2);

% x_hat = [longDist; longVelo] => 2x1
% x_hat_n1 = [1, dt; 0, 1] x_hat => 2x2*2x1

dt = .01;
F = [1 dt;
     0 1];

P = .1*eye(2);
x_hat = [0; 0];

I = eye(2);

% filter values for evaluation
x_f = zeros(500,1);
xv_f = zeros(500, 1);

for t = 1:500
    % Prediction
    x_hat = F*x_hat;

    P = F*P*F' + Q;

    % Update
    % Kalman Variable
    K = P*H' / (H*P*H' + R);
    x_hat = x_hat + K*(z(t,:)' - H*x_hat);

    % estimate covariance
    P = (I - K*H)*P*(I-K*H)' + K*R*K';

    x_f(t) = x_hat(1);
    xv_f(t) = x_hat(2);
end

plot(x(:,1), "DisplayName", "Actual"); hold on; grid on;
plot(z(:,1), "DisplayName","Distance Sensor Data");
plot(x_f, "DisplayName","FilterOutput")
legend; hold off;
