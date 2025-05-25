function [optimal_path, num_points, best_idx, candidates_cost] = collision_check_func(valid_idx, candidates_cost, candidate_paths_cartesian)
    % 열 벡터로 변환하여 오류 방지
    valid_idx = valid_idx(:);

    % 전체 경로 개수
    num_paths = length(candidates_cost);

    % 유효하지 않은 인덱스 찾기 (valid_idx가 비어 있어도 문제 없음)
    all_idx = (1:num_paths).';
    invalid_idx = setdiff(all_idx, valid_idx);

    % 유효하지 않은 인덱스에 penalty 부여
    penalty = 2e6;
    candidates_cost(invalid_idx) = candidates_cost(invalid_idx) + penalty;

    % 최소 cost 경로 선택
    [~, best_idx] = min(candidates_cost);

    % 최종 선택 경로 반환
    optimal_path = candidate_paths_cartesian(:, :, best_idx);
    num_points = size(optimal_path, 1);
end