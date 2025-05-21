function [steering_angle, x_lookahead] = adaptive_pure_pursuit_from_poly_fnc(poly_coeff, velocity)
% Adaptive Pure Pursuit with dynamic lookahead based on velocity and curvature
% 입력:
%   poly_coeff : 경로 다항식 계수 [1x4] (y = ax^3 + bx^2 + cx + d)
%   velocity   : 자차 속도 [m/s]
% 출력:
%   steering_angle : 조향각 [rad]
%   x_lookahead    : lookahead 거리 [m]

    % 차량 파라미터
    L = 2.9;  % 차량 휠베이스 [m]

    % Lookahead 계산 파라미터
    Lxd = 2.5;       % 기본 lookahead [m]
    kv = 0.8;        % 속도 계수
    kc = 0.05;       % 곡률 보정 계수
    rad2deg = 180 / pi;

    % 곡률 평가 위치 (예: x=3.0)에서 곡률 추정
    x_for_curv = 3.0;
    dy_dx  = polyval(polyder(poly_coeff), x_for_curv);
    d2y_dx2 = polyval(polyder(polyder(poly_coeff)), x_for_curv);
    curvature = d2y_dx2 / (1 + dy_dx^2)^(3/2);  % 곡률

    % lookahead 거리 계산
    x_lookahead = Lxd + kv * velocity - kc * abs(curvature * rad2deg);
    x_lookahead = max(2.0, x_lookahead);  % 최소 거리 제한

    % lookahead 위치에서 목표점 좌표
    y_target = polyval(poly_coeff, x_lookahead);

    % 좌표 기반 조향각 계산 (Pure Pursuit 방식)
    alpha = atan2(y_target, x_lookahead);  % heading error
    steering_angle = atan2(2 * L * sin(alpha), x_lookahead);
end
