%standard deviation of measurements
R_x = 0.0045;
R_v = 0.0011;

%generate random process data with given noise

x = repmat([8 0],500,1);


%generate random measuremet data with given noise
R = [R_x^2 0; 0 R_v^2];
z = [x(:,1) + randn(500,1)*R_x x(:,2) + randn(500,1)*R_v];

baseline_x_rmse = sqrt(sum((x(:,1) - z(:,1)).^2)/500);
baseline_xv_rmse = sqrt(sum((x(:,2) - z(:,2)).^2)/500);