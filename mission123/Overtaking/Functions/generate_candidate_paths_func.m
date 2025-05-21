function candidate_paths = generate_candidate_paths_func(route, route1_frenet, route4_frenet, ego_s, ego_d)

    global_s = route(:,1);
    global_d = route(:,2);
    
    % lane_width를 고정값으로 가정
    lane_width = 6;
    num_lateral_paths = 9;  % 반드시 홀수 (중앙값이 0이 되도록)
    
    % d
    % 왼쪽이 양수
    % 오른쪽이 음수
    
    % 중앙값이 0이 되도록 등간격 리스트 생성
    half_range = lane_width/2;
    step = 2 * half_range / (num_lateral_paths - 1);
    deviation_list = -half_range : step : half_range;
    
    if isequal(route, route1_frenet)
        if ego_d > 0
            deviation_list(deviation_list > 0) = 0; % 차량이 왼쪽 차선의 바깥쪽에 붙은 경우 중앙~오른쪽만 허용
        end 
    end
    
    if isequal(route, route4_frenet)
        if ego_d < 0
            deviation_list(deviation_list < 0) = 0;  % 차량이 오른쪽 차선의 바깥쪽에 붙은 경우 차선은 중앙~왼쪽만 허용
        end
    end
    
    % 경로 설정 파라미터
    lookahead_index  = 5;          % lookahead_index
    num_points = 20;
    
    candidate_paths = generate_candidate_paths_func( ...
        global_s, global_d, ego_s, ego_d, deviation_list, lookahead_index , num_points);
end