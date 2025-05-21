function [route_waypoint, route_frenet] = select_lane_func(lane_id, route1_waypoint, ...
    route1_frenet, route2_waypoint, route2_frenet, route3_waypoint, route3_frenet, ...
    route4_waypoint, route4_frenet, route5_waypoint, route5_frenet)

    switch lane_id
        case 1
            route_waypoint = route1_waypoint;
            route_frenet = route1_frenet;
        case 2
            route_waypoint = route2_waypoint;
            route_frenet = route2_frenet;
        case 3
            route_waypoint = route3_waypoint;
            route_frenet = route3_frenet;
        case 4
            route_waypoint = route4_waypoint;
            route_frenet = route4_frenet;
        case 5
            route_waypoint = route5_waypoint;
            route_frenet = route5_frenet;
        otherwise
            route_waypoint = route2_waypoint;
            route_frenet = route2_frenet;
    end
end