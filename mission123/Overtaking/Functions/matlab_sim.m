% 데이터 로드
load('E_X.mat');    
load('E_Y.mat');    
load('oc.mat');     
load('route1_waypoint.mat');
load('route2_waypoint.mat');
load('route3_waypoint.mat');
load('candidate_oc.mat');
load('obstacle.mat');  % 장애물 차량


% 길이 설정
N = length(E_X);
oc_frame_count = size(oc, 3);
[~,~,num_candidates,candidate_frame_count] = size(candidate_oc);
[obs_count, ~, obs_frame_count] = size(obstacle);  % 8 x 2 x 36434

% 전체 범위 계산
all_x = [E_X(:); reshape(oc(:,1,:), [], 1); reshape(candidate_oc(:,1,:,:), [], 1); reshape(obstacle(:,1,:), [], 1);
         route1_waypoint(:,1); route2_waypoint(:,1); route3_waypoint(:,1)];
all_y = [E_Y(:); reshape(oc(:,2,:), [], 1); reshape(candidate_oc(:,2,:,:), [], 1); reshape(obstacle(:,2,:), [], 1);
         route1_waypoint(:,2); route2_waypoint(:,2); route3_waypoint(:,2)];

% Figure 설정
% figure('Position', [100, 100, 1200, 800]); % [left, bottom, width, height]
figure;
hold on;

% 1. Reference Waypoints
plot(route1_waypoint(:,1), route1_waypoint(:,2), 'b-', 'LineWidth', 1.2);
plot(route2_waypoint(:,1), route2_waypoint(:,2), 'g-', 'LineWidth', 1.2);
plot(route3_waypoint(:,1), route3_waypoint(:,2), 'c-', 'LineWidth', 1.2);

% 2. Path (oc)
h_path = plot(nan, nan, 'r-', 'LineWidth', 2.0);

% 3. 후보 경로 (candidate_oc)
h_candidates = gobjects(num_candidates,1);
candidate_color = [0.5, 0.5, 0.5];
for k = 1:num_candidates
    h_candidates(k) = plot(nan, nan, '-', 'Color', candidate_color, 'LineWidth', 0.8);
end

% 4. 차량 위치 및 궤적
h_vehicle = plot(E_X(1), E_Y(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
h_trace = plot(E_X(1), E_Y(1), 'r-', 'LineWidth', 1.5);

% 5. 장애물 차량 핸들 (검정색 네모)
h_obstacles = gobjects(obs_count, 1);
for j = 1:obs_count
    h_obstacles(j) = plot(nan, nan, 'ks', 'MarkerSize', 6, 'MarkerFaceColor', 'k');  % 검정 네모
end

% Plot 설정
title('Real-time Vehicle, Path, and Obstacles Visualization');
xlabel('X Position'); ylabel('Y Position');
legend_entries = {'Route 1', 'Route 2', 'Route 3', 'Path (oc)'};
for k = 1:num_candidates
    legend_entries{end+1} = ['Candidate ' num2str(k)];
end
legend_entries = [legend_entries, {'Vehicle Position', 'Vehicle Trace', 'Obstacles'}];
legend(legend_entries{:});
axis equal;
grid on;
xlim([min(all_x)-1, max(all_x)+1]);
ylim([min(all_y)-1, max(all_y)+1]);


% 실시간 루프
for i = 2:50:N
    % 차량 위치 업데이트
    set(h_vehicle, 'XData', E_X(i), 'YData', E_Y(i));
    set(h_trace, 'XData', E_X(1:i), 'YData', E_Y(1:i));
    
    % oc 경로
    if i <= oc_frame_count
        oc_x = oc(:,1,i);
        oc_y = oc(:,2,i);
        set(h_path, 'XData', oc_x, 'YData', oc_y);
    end

    % 후보 경로
    if i <= candidate_frame_count
        for k = 1:num_candidates
            cand_x = candidate_oc(:,1,k,i);
            cand_y = candidate_oc(:,2,k,i);
            set(h_candidates(k), 'XData', cand_x, 'YData', cand_y);
        end
    end

    % 장애물 차량
    if i <= obs_frame_count
        for j = 1:obs_count
            obs_x = obstacle(j,1,i);
            obs_y = obstacle(j,2,i);
            set(h_obstacles(j), 'XData', obs_x, 'YData', obs_y);
        end
    end

    drawnow;
    pause(0.03);
end
