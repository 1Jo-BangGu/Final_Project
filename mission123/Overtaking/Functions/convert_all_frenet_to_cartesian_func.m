function [cartesian_path, num_points] = convert_all_frenet_to_cartesian_func(optimal_path, route_waypoints)
    % optimal_path: (num_points x 2 x num_paths) Frenet (s, d) 배열
    % route_waypoints: (N x 2) 기준 경로 (x, y)
    % 반환값 cartesian_path: (num_points x 4 x num_paths) [x, y, yaw, kappa]
    
    [num_points, ~, num_paths] = size(optimal_path);
    cartesian_path = zeros(num_points, 4, num_paths);
    
    map_x = route_waypoints(:,1);
    map_y = route_waypoints(:,2);
    
    % 기준 경로의 누적 거리 s 계산
    map_s = zeros(length(map_x),1);
    for i = 2:length(map_x)
        map_s(i) = map_s(i-1) + hypot(map_x(i)-map_x(i-1), map_y(i)-map_y(i-1));
    end
    
    for k = 1:num_paths
        % (s, d) → (x, y)
        for i = 1:num_points
            s = optimal_path(i,1,k);
            d = optimal_path(i,2,k);
            [x, y] = cartesian_converter(s, d, map_x, map_y, map_s);
            cartesian_path(i,1,k) = x;
            cartesian_path(i,2,k) = y;
        end
    
        % yaw 계산
        for i = 1:num_points-1
            dx = cartesian_path(i+1,1,k) - cartesian_path(i,1,k);
            dy = cartesian_path(i+1,2,k) - cartesian_path(i,2,k);
            cartesian_path(i,3,k) = atan2(dy, dx);
        end
        cartesian_path(num_points,3,k) = cartesian_path(num_points-1,3,k); % 마지막 yaw
    
        % kappa(곡률) 계산
        for i = 1:num_points-1
            dyaw = cartesian_path(i+1,3,k) - cartesian_path(i,3,k);
            % -pi~pi wrap
            dyaw = atan2(sin(dyaw), cos(dyaw));
            ds = hypot(cartesian_path(i+1,1,k) - cartesian_path(i,1,k), cartesian_path(i+1,2,k) - cartesian_path(i,2,k));
            if ds ~= 0
                cartesian_path(i,4,k) = dyaw / ds;
            else
                cartesian_path(i,4,k) = 0;
            end
        end
        cartesian_path(num_points,4,k) = cartesian_path(num_points-1,4,k); % 마지막 kappa
    end
end