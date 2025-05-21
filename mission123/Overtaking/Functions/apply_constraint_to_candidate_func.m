function valid_idx = apply_constraint_to_candidate_func(candidate_paths_cartesian, obs_xy)
    % candidate_paths_cartesian: [num_points x 4 x num_candidates] (4번째 열이 kappa)
    % kappa_threshold: 허용 최대 곡률 값 (예: 0.2)
    % obs_xy: [M x 2] 장애물의 (x, y) 좌표 (Cartesian)
    % col_check_d: 충돌 판정 거리 (예: 2.0)
    % 반환값 valid_idx: 조건을 만족하는 후보 경로 인덱스 (벡터)

    kappa_threshold = 0.2;
    col_check_d = 0.5;

    [num_points, ~, num_candidates] = size(candidate_paths_cartesian);
    valid_idx = [];
    
    for i = 1:num_candidates
        kappa = candidate_paths_cartesian(:,4,i);
        % 곡률 조건
        kappa_ok = all(abs(kappa) <= kappa_threshold);
    
        % 충돌 조건
        collision = false;
        for j = 1:size(obs_xy,1)
            obs_x = obs_xy(j,1);
            obs_y = obs_xy(j,2);
            dx = candidate_paths_cartesian(:,1,i) - obs_x;
            dy = candidate_paths_cartesian(:,2,i) - obs_y;
            d2 = dx.^2 + dy.^2;
            if any(d2 <= col_check_d.^2)
                collision = true;
                break;
            end
        end
    
        % AND 조건
        if kappa_ok && ~collision
            valid_idx = [valid_idx, i];
        end
    end
end
