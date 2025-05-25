function [local_waypoints_, goal_reached, near_goal_zone] = global_to_local_waypoints_fnc(path_len, waypoints, vehicle_position, head, success_flag)
% 로컬 좌표계 변환 + 도착 판별 + near_goal_zone 신호 생성
% 입력:
%   path_len         : 유효한 경로 길이
%   waypoints        : 전체 경로 [400 x 2]
%   vehicle_position : [x, y]
%   head             : 자차 yaw
%   success_flag     : 경로 생성 성공 여부
% 출력:
%   local_waypoints_ : 로컬 변환 경로 [400 x 2]
%   goal_reached     : 도착 여부 (1m 이내)
%   near_goal_zone   : 남은 포인트가 4개 이하

    local_waypoints_ = zeros(500, 2);  % Simulink 고정 크기 호환
    goal_reached = false;
    near_goal_zone = false;

    if ~success_flag || path_len < 1
        return;
    end

    x_ego = vehicle_position(1);
    y_ego = vehicle_position(2);

    % --- 도착 여부 판단 (절대 거리 기준) ---
    goal_threshold = 4.0;  % [m]
    goal_point = waypoints(path_len, :);
    distance_to_goal = hypot(goal_point(1) - x_ego, goal_point(2) - y_ego);
    if distance_to_goal < goal_threshold
        goal_reached = true;
    end

    % --- 가장 가까운 waypoint 인덱스 찾기 ---
    min_dist = inf;
    closest_idx = 1;
    for i = 1:path_len
        dist = hypot(waypoints(i,1) - x_ego, waypoints(i,2) - y_ego);
        if dist < min_dist
            min_dist = dist;
            closest_idx = i;
        end
    end

    % --- near_goal_zone 판단 (남은 waypoint 개수 기준) ---
    remaining_points = path_len - closest_idx + 1;
    if remaining_points <= 30
        near_goal_zone = true;
    end

    % --- 로컬 좌표계로 변환 ---
    idx = 1;
    for i = closest_idx:path_len
        dx = waypoints(i,1) - x_ego;
        dy = waypoints(i,2) - y_ego;

        x_local = dx * cos(-head) - dy * sin(-head);
        y_local = dx * sin(-head) + dy * cos(-head);

        local_waypoints_(idx, :) = [x_local, y_local];
        idx = idx + 1;
    end
end
