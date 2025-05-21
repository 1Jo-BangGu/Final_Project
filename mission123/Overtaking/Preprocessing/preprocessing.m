% route 별 waypoint raw 데이터
route1_waypoint_raw = [route1_waypoint_x.Data, route1_waypoint_y.Data];
route2_waypoint_raw = [route2_waypoint_x.Data, route2_waypoint_y.Data];
route3_waypoint_raw = [route3_waypoint_x.Data, route3_waypoint_y.Data];
route4_waypoint_raw = [route4_waypoint_x.Data, route4_waypoint_y.Data];
route5_waypoint_raw = [route5_waypoint_x.Data, route5_waypoint_y.Data];

% route 별 waypoint 필터링
route1_waypoint_temp = [];
route2_waypoint_temp = [];
route3_waypoint_temp = [];
route4_waypoint_temp = [];
route5_waypoint_temp = [];

for i=1:length(route1_waypoint_raw)
    add_point = true;
    for j=1:size(route1_waypoint_temp,1)
        if sqrt((route1_waypoint_raw(i,1)-route1_waypoint_temp(j,1))^2 + (route1_waypoint_raw(i,2)-route1_waypoint_temp(j,2))^2) < 3
            add_point = false;
            break;
        end
    end
    if add_point
        route1_waypoint_temp = [route1_waypoint_temp; route1_waypoint_raw(i,:)];
    end
end

for i=1:length(route2_waypoint_raw)
    add_point = true;
    for j=1:size(route2_waypoint_temp,1)
        if sqrt((route2_waypoint_raw(i,1)-route2_waypoint_temp(j,1))^2 + (route2_waypoint_raw(i,2)-route2_waypoint_temp(j,2))^2) < 3
            add_point = false;
            break;
        end
    end
    if add_point
        route2_waypoint_temp = [route2_waypoint_temp; route2_waypoint_raw(i,:)];
    end
end

for i=1:length(route3_waypoint_raw)
    add_point = true;
    for j=1:size(route3_waypoint_temp,1)
        if sqrt((route3_waypoint_raw(i,1)-route3_waypoint_temp(j,1))^2 + (route3_waypoint_raw(i,2)-route3_waypoint_temp(j,2))^2) < 3
            add_point = false;
            break;
        end
    end
    if add_point
        route3_waypoint_temp = [route3_waypoint_temp; route3_waypoint_raw(i,:)];
    end
end

for i=1:length(route4_waypoint_raw)
    add_point = true;
    for j=1:size(route4_waypoint_temp,1)
        if sqrt((route4_waypoint_raw(i,1)-route4_waypoint_temp(j,1))^2 + (route4_waypoint_raw(i,2)-route4_waypoint_temp(j,2))^2) < 4
            add_point = false;
            break;
        end
    end
    if add_point
        route4_waypoint_temp = [route4_waypoint_temp; route4_waypoint_raw(i,:)];
    end
end

for i=1:length(route5_waypoint_raw)
    add_point = true;
    for j=1:size(route5_waypoint_temp,1)
        if sqrt((route5_waypoint_raw(i,1)-route5_waypoint_temp(j,1))^2 + (route5_waypoint_raw(i,2)-route5_waypoint_temp(j,2))^2) < 5
            add_point = false;
            break;
        end
    end
    if add_point
        route5_waypoint_temp = [route5_waypoint_temp; route5_waypoint_raw(i,:)];
    end
end

% 전체 waypoint 필터링
waypoint_temp = [route1_waypoint_temp; route2_waypoint_temp; route3_waypoint_temp; route4_waypoint_temp; route5_waypoint_temp;];
waypoint = [];

for i=1:length(waypoint_temp(:,1))
    add_point = true;
    for j=1:size(waypoint,1)
        if sqrt((waypoint_temp(i,1)-waypoint(j,1))^2 + (waypoint_temp(i,2)-waypoint(j,2))^2) < 3
            add_point = false;
            break;
        end
    end
    if add_point
        waypoint = [waypoint; waypoint_temp(i,:)];
    end
end

% route 별 필터링 (전체 waypoint에 존재하는지, 다른 route에 동일한 데이터가 존재하는지)
route1_waypoint = [];
route2_waypoint = [];
route3_waypoint = [];
route4_waypoint = [];
route5_waypoint = [];
redundant = [0,0];

for i=1:length(route1_waypoint_temp)
    if ismember(route1_waypoint_temp(i,:),waypoint,'rows')==1 && ismember(route1_waypoint_temp(i,:),redundant,'rows')~=1
        route1_waypoint = [route1_waypoint; route1_waypoint_temp(i,:)];
        redundant = [redundant; route1_waypoint_temp(i,:)];
    end
end
for i=1:length(route2_waypoint_temp)
    if ismember(route2_waypoint_temp(i,:),waypoint,'rows')==1 && ismember(route2_waypoint_temp(i,:),redundant,'rows')~=1
        route2_waypoint = [route2_waypoint; route2_waypoint_temp(i,:)];
        redundant = [redundant; route2_waypoint_temp(i,:)];
    end
end
for i=1:length(route3_waypoint_temp)
    if ismember(route3_waypoint_temp(i,:),waypoint,'rows')==1 && ismember(route3_waypoint_temp(i,:),redundant,'rows')~=1
        route3_waypoint = [route3_waypoint; route3_waypoint_temp(i,:)];
        redundant = [redundant; route3_waypoint_temp(i,:)];
    end
end
for i=1:length(route4_waypoint_temp)
    if ismember(route4_waypoint_temp(i,:),waypoint,'rows')==1 && ismember(route4_waypoint_temp(i,:),redundant,'rows')~=1
        route4_waypoint = [route4_waypoint; route4_waypoint_temp(i,:)];
        redundant = [redundant; route4_waypoint_temp(i,:)];
    end
end
for i=1:length(route5_waypoint_temp)
    if ismember(route5_waypoint_temp(i,:),waypoint,'rows')==1 && ismember(route5_waypoint_temp(i,:),redundant,'rows')~=1
        route5_waypoint = [route5_waypoint; route5_waypoint_temp(i,:)];
        redundant = [redundant; route5_waypoint_temp(i,:)];
    end
end

route1_frenet = routes_to_frenets(route1_waypoint, 1);
route2_frenet = routes_to_frenets(route2_waypoint, 1);
route3_frenet = routes_to_frenets(route3_waypoint, 1);
route4_frenet = routes_to_frenets(route4_waypoint, 1);
route5_frenet = routes_to_frenets(route5_waypoint, 1);