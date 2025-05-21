function candidate_paths = generate_candidate_paths_func(global_s, global_d, ego_s, ...
    ego_d, deviation_list, index_offset, num_points)
    % global_s, global_d : 전역 경로의 frenet 좌표 (벡터)
    % ego_s, ego_d       : ego 차량의 frenet 좌표 (스칼라)
    % deviation_list     : d 방향 deviation 리스트 (예: [-2 -1.5 ... 2])
    % index_offset       : ego 위치에서 몇 인덱스 이후를 terminal로 할지 (예: 10)
    % num_points         : 각 후보 경로의 포인트 개수 (예: 50)
    %
    % 반환값: candidate_paths (cell array, 각 cell에 [num_points x 2] (s, d) 배열)

    N = length(deviation_list);
    candidate_paths = zeros(num_points, 2, N);

    % 1. ego 위치와 가장 가까운 점 찾기
    [~, closest_idx] = min(abs(global_s - ego_s));

    % 2. 기준점 인덱스 (10개 이후)
    base_idx = closest_idx + index_offset;
    if base_idx > length(global_s)
        base_idx = length(global_s);
    end
    s_base = global_s(base_idx);
    d_base = global_d(base_idx);

    % 3. 각 deviation에 대해 terminal point 생성 및 경로 생성
    for i = 1:N
        terminal_s = s_base;
        terminal_d = d_base + deviation_list(i);

        % 4. quintic curve 계수 생성 (초기 속도/가속도는 0으로 가정)
        coeffs = quintic_curve_generate(ego_s, ego_d, 0, 0, terminal_s, terminal_d);
        
        % 5. s 값 균등 분할
        s_vals = linspace(ego_s, terminal_s, num_points);

        % 6. d 값 계산 (polyval은 계수 역순)
        d_vals = polyval(flip(coeffs), s_vals);

        candidate_paths(:,1,i) = s_vals(:);
        candidate_paths(:,2,i) = d_vals(:);
    end
end