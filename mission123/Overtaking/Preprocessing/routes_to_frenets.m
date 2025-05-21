function FrenetStates = routes_to_frenets(waypoints, interval)
    persistent RefPathFrenet
    
    N = size(waypoints, 1);
    sampledIdx = 1:interval:N;

    if isempty(RefPathFrenet)
        RefPathFrenet = referencePathFrenet(waypoints);
    elseif ~isequal(RefPathFrenet.Waypoints, waypoints)
        RefPathFrenet.Waypoints = waypoints;
    end

    FrenetStates = zeros(length(sampledIdx), 6); % [s d theta kappa v a]
    
    for i = 1:length(sampledIdx)
        idx = sampledIdx(i);
        pos = waypoints(idx, 1:2);
    
        % yaw 계산
        if size(waypoints,2) >= 3
            yaw = waypoints(idx, 3);
        elseif idx < N
            delta = waypoints(idx+1,1:2) - pos;
            yaw = atan2(delta(2), delta(1));
        else
            yaw = 0;
        end
    
        cartState = [pos yaw 0 0 0];
        frenetState = global2frenet(RefPathFrenet, cartState);
        FrenetStates(i, :) = frenetState;
    end
end