function [path_out, success_flag, path_len] = rrt_star_simulink_fnc(traffic_info, Ego_Global_X, Ego_Global_Y)
%#codegen

persistent path_cached path_len_cached success_cached 
if isempty(path_cached)
    path_cached = zeros(400, 2);
    path_len_cached = 0;
    success_cached = false;
end

path_out = path_cached;
path_len = path_len_cached;
success_flag = success_cached;

if success_cached
    return;
end

MAX_NODES = 3000;
MAX_PATH_LEN = 400;
goal_radius = 12;
goal_sample_rate = 0.1;
eta = 3.0;

start = [Ego_Global_X, Ego_Global_Y];
goal  =  [17, -10.1];
vehicle_size = [1.97, 4.47];

map_boundary = [5.5, -4, 47.5, -3.9, 47.5, -44.9, 5.5, -44.9];
x_vals = map_boundary(1:2:end);
y_vals = map_boundary(2:2:end);
x_min = min(x_vals); x_max = max(x_vals);
y_min = min(y_vals); y_max = max(y_vals);

if mod(length(traffic_info), 5) ~= 0
    obstacles = zeros(0,5);
else
    num_obj = int32(floor(length(traffic_info)/5));
    data = reshape(traffic_info, [5, num_obj])';
    obstacles = [data(:,1), data(:,2), data(:,5), data(:,4), data(:,3)];
end

empty_node = struct('pos', zeros(1,2), 'cost', 0, 'parent', int32(0));
nodes = repmat(empty_node, MAX_NODES, 1);
nodes(1).pos = reshape(start,1,2);
node_count = int32(1);
goal_node = int32(0);

for iter = 1:MAX_NODES
    r = rand;
    if r < goal_sample_rate
        sample = goal;
    elseif r < 0.6
        sample = [x_min + rand*(x_max - x_min), y_min + rand*(y_max - y_min)];
    else
        sample = [x_min + rand*(x_max - x_min), -15 + rand * (-5)];
    end
    sample(1) = min(max(sample(1), x_min), x_max);
    sample(2) = min(max(sample(2), y_min), y_max);

    nearest_idx = int32(1);
    min_dist = norm(nodes(1).pos - sample);
    for i = 2:node_count
        d = norm(nodes(i).pos - sample);
        if d < min_dist
            min_dist = d;
            nearest_idx = int32(i);
        end
    end

    direction = sample - nodes(nearest_idx).pos;
    dist = norm(direction);
    if dist > eta
        direction = direction / dist * eta;
    end
    new_pos = nodes(nearest_idx).pos + direction;
    new_pos = reshape(new_pos,1,2);

    if check_collision(nodes(nearest_idx).pos, new_pos, vehicle_size, obstacles)
        continue;
    end

    node_count = node_count + 1;
    if node_count > MAX_NODES
        break;
    end
    nodes(node_count).pos = new_pos;
    nodes(node_count).cost = nodes(nearest_idx).cost + dist;
    nodes(node_count).parent = nearest_idx;

    if norm(new_pos - goal) < goal_radius
        goal_node = node_count;
        break;
    end
end

if goal_node > 0
    idx = goal_node;
    i = 0;
    temp_path = zeros(MAX_PATH_LEN, 2);
    while idx > 0 && i < MAX_PATH_LEN
        i = i + 1;
        temp_path(i,:) = reshape(nodes(idx).pos,1,2);
        idx = nodes(idx).parent;
    end

    flipped = flipud(temp_path(1:i,:));

    % Hybrid A* 직선 후처리
    pre_steps  = 12;
    post_steps = 6;
    step_dist  = 0.5;
    last_pos = flipped(end, :);

    if last_pos(1) < goal(1)
        hybrid_x = goal(1) - 0.9;
    else
        hybrid_x = goal(1) + 0.9;
    end

    if goal(2) > -36.5
        pre_path = [ repmat(hybrid_x, pre_steps,1), ...
                     goal(2) - step_dist*(pre_steps:-1:1)' ];
        post_path = [ repmat(hybrid_x, post_steps,1), ...
                      goal(2) + step_dist*(1:post_steps)' ];
    else
        pre_path = [ repmat(hybrid_x, pre_steps,1), ...
                     goal(2) + step_dist*(pre_steps:-1:1)' ];
        post_path = [ repmat(hybrid_x, post_steps,1), ...
                      goal(2) - step_dist*(1:post_steps)' ];
    end

    hybrid_path = [pre_path; goal; post_path];
    raw_path    = [flipped; hybrid_path];
    path_interp = smooth_path(raw_path, 0.5);
    path_interp = smooth_moving_average(path_interp, 35);

    path_len    = size(path_interp,1);
    path_out    = zeros(MAX_PATH_LEN, 2);
    path_out(1:path_len, :) = path_interp;
    success_flag = true;

    path_cached     = path_out;
    path_len_cached = path_len;
    success_cached  = true;
end

end

%% === 보조 함수 ===
function collision = check_collision(p1, p2, vehicle_size, obs_list)
    collision = false;
    steps = 10;
    for t = linspace(0,1,steps)
        pt = (1-t)*p1 + t*p2;
        yaw = atan2(p2(2)-p1(2), p2(1)-p1(1));
        veh = get_bbox(pt, yaw, vehicle_size);
        for i = 1:size(obs_list,1)
            if check_single_collision(veh, obs_list(i,:))
                collision = true; return;
            end
        end
    end
end

function is_collide = check_single_collision(veh_box, obs)
    yaw_obs = obs(5);
    rear_center = obs(1:2);
    length_obs = obs(4);
    width_obs = obs(3);
    fwd_vec = [cos(yaw_obs), sin(yaw_obs)];
    center = rear_center + (length_obs/2) * fwd_vec;
    obs_box = get_bbox(center, yaw_obs, [width_obs, length_obs]);
    is_collide = box_overlap(veh_box, obs_box);
end

function box = get_bbox(center, yaw, size)
    w = size(1); h = size(2);
    dx = [ w/2, -w/2, -w/2,  w/2];
    dy = [ h/2,  h/2, -h/2, -h/2];
    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    temp = R * [dx; dy];
    box = temp + center';
end

function overlap = box_overlap(b1, b2)
    overlap = true;
    axes = get_axes(b1, b2);
    for i = 1:size(axes,2)
        proj1 = axes(:,i)' * b1;
        proj2 = axes(:,i)' * b2;
        if max(proj1) < min(proj2) || max(proj2) < min(proj1)
            overlap = false;
            return;
        end
    end
end

function axes = get_axes(b1, b2)
    edges = [b1(:,2)-b1(:,1), b1(:,3)-b1(:,2)];
    normals = [-edges(2,:); edges(1,:)];
    axes = normals ./ vecnorm(normals);
end

function path_smooth = smooth_path(path_raw, step_size)
    if size(path_raw, 1) < 2
        path_smooth = path_raw;
        return;
    end
    dist = [0; cumsum(vecnorm(diff(path_raw), 2, 2))];
    query = 0:step_size:dist(end);
    if query(end) < dist(end)
        query(end+1) = dist(end);
    end
    x_interp = spline(dist, path_raw(:,1), query);
    y_interp = spline(dist, path_raw(:,2), query);
    path_smooth = [x_interp', y_interp'];
end

function smoothed = smooth_moving_average(path, windowSize)
    N = size(path,1);
    half = floor(windowSize/2);
    smoothed = zeros(N,2);
    for i = 1:N
        if i == 1 || i == N
            smoothed(i,:) = path(i,:);
        else
            i0 = max(1, i-half);
            i1 = min(N, i+half);
            smoothed(i,:) = mean(path(i0:i1,:), 1);
        end
    end
end