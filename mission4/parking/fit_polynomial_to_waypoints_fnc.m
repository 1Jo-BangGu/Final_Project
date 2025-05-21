function poly_coeff = fit_polynomial_to_waypoints_fnc(local_waypoints)
% 전방 포인트(x > 0) 중 y값 차이가 10 이하이며, y값이 -30~30 사이인 포인트만 사용

    % 1차 필터링: x > 0
    valid_idx = local_waypoints(:,1) > 0;
    filtered = local_waypoints(valid_idx, :);

    % 2차 필터링: y in [-8, 8]
    y_range_idx = abs(filtered(:,2)) <= 8;
    filtered = filtered(y_range_idx, :);

    % 앞쪽 30개만 사용
    N = min(30, size(filtered, 1));
    x = filtered(1:N, 1);
    y = filtered(1:N, 2);

    % 최소 포인트 수 확인 후 3차 다항식 피팅
    if length(x) >= 4
        poly_coeff = polyfit(x, y, 3);
    else
        poly_coeff = zeros(1, 4);
    end
end