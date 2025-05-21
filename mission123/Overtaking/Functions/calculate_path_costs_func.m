function path_costs = calculate_path_costs_func(candidate_paths, opt_d, target_speed, dt)
    %#codegen
    K_J = 0.01; K_T = 0.1; K_D = 100.0;
    K_V = 1.0; K_LAT = 10.0; K_LON = 1.0;
    
    [num_points, ~, num_paths] = size(candidate_paths);
    path_costs = zeros(num_paths,1);
    
    for i = 1:num_paths
        s_vals = squeeze(candidate_paths(:,1,i));
        d_vals = squeeze(candidate_paths(:,2,i));
        
        s_d = diff(s_vals) / dt;
        s_dd = diff(s_d) / dt;
        s_ddd = diff(s_dd) / dt;
    
        d_d = diff(d_vals) / dt;
        d_dd = diff(d_d) / dt;
        d_ddd = diff(d_dd) / dt;
    
        J_lat = sum(d_ddd.^2);
        J_lon = sum(s_ddd.^2);
    
        % ✅ 개선된 부분
        d_diff = sum((d_vals - opt_d).^2) / num_points;
        v_diff = (target_speed - s_d(end))^2;
    
        c_lat = K_J * J_lat + K_T * num_points*dt + K_D * d_diff;
        c_lon = K_J * J_lon + K_T * num_points*dt + K_V * v_diff;
    
        path_costs(i) = K_LAT * c_lat + K_LON * c_lon;
    end
end
