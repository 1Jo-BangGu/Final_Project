function [s_path, d_path, best_idx] = select_lowest_cost_path(candidate_paths, candidates_cost)
% candidate_paths: (num_points x 2 x num_paths)
% candidates_cost: (num_paths x 1)
% s_path, d_path: 최적 경로의 (s,d) trajectory

    % 1. 최소 비용 경로 인덱스 찾기
    [~, best_idx] = min(candidates_cost);

    % 2. 해당 인덱스의 (s, d) trajectory 추출
    s_path = squeeze(candidate_paths(:, 1, best_idx)); % s(t)
    d_path = squeeze(candidate_paths(:, 2, best_idx)); % d(t)
end