function target_velocity = longitudinal_speed_control_from_poly_fnc(poly_coeff, goal_reached, near_goal_zone, success_flag, lookahead)
    V_max = 7.0;
    V_min = 2.0;
    x_for_curv = lookahead;

    % 곡률 계산
    dy_dx   = polyval(polyder(poly_coeff), x_for_curv);
    d2y_dx2 = polyval(polyder(polyder(poly_coeff)), x_for_curv);
    kappa = d2y_dx2 / (1 + dy_dx^2)^(3/2);

    % 속도 계산
    time_horizon = 3.0;
    v_from_lookahead = lookahead / time_horizon;
    v_from_curvature = V_max * exp(-5.0 * abs(kappa));
    v = min(v_from_lookahead, v_from_curvature);

    % 기본 보정
    if ~success_flag
        v = 0;
    else
        v = max(V_min, v);
    end

    if near_goal_zone
        v = min(v, 1.0);
    end

    if goal_reached
        v = 0;
    end

    % ======== 저역통과 필터 (optional) =========
    persistent prev_v
    if isempty(prev_v)
        prev_v = v;
    end

    alpha = 0.95;  % 0.85~0.95 추천 (1에 가까울수록 부드럽고 느림)
    target_velocity = alpha * prev_v + (1 - alpha) * v;
    prev_v = target_velocity;
end
