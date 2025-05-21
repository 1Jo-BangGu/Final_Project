function [x, y] = cartesian_converter(s, d, map_x, map_y, map_s)

    % 반복 순환 고려 (맵 길이 넘어가면 순환)
    if s >= map_s(end)
        s = mod(s, map_s(end));
    end

    % s보다 큰 구간 찾기
    prev_wp = 1;
    while (s > map_s(prev_wp+1)) && (prev_wp < length(map_s)-1)
        prev_wp = prev_wp + 1;
    end

    next_wp = prev_wp + 1;

    dx = map_x(next_wp) - map_x(prev_wp);
    dy = map_y(next_wp) - map_y(prev_wp);
    heading = atan2(dy, dx);

    % 중심선 상의 s 지점 (투영 위치)
    seg_s = s - map_s(prev_wp);
    seg_x = map_x(prev_wp) + seg_s * cos(heading);
    seg_y = map_y(prev_wp) + seg_s * sin(heading);

    % d만큼 수직 방향 offset 추가
    perp_heading = heading + pi/2;
    x = seg_x + d * cos(perp_heading);
    y = seg_y + d * sin(perp_heading);
end
