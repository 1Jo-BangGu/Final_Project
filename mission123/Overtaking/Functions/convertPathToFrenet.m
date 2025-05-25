function EgoFrenet = convertPathToFrenet(waypoints, interval, egoState)
    % function [FrenetStates, EgoFrenet] = convertPathToFrenet(waypoints, interval, egoState)
    % waypoints : Nx2 or Nx3 matrix [x y] or [x y heading]
    % interval  : 경로 샘플 간격
    % egoState  : 1x6 [x y yaw kappa v a] 전역 상태

    coder.inline('never');
    persistent RefPathFrenet

    % 기준 경로 객체 (한 번만 생성)
    if isempty(RefPathFrenet)
        RefPathFrenet = referencePathFrenet(waypoints);
    elseif ~isequal(RefPathFrenet.Waypoints, waypoints)
        RefPathFrenet.Waypoints = waypoints;
    end

    egoState = reshape(double(egoState), 1, []);

    % Ego 차량 상태도 Frenet 변환
    EgoFrenet = global2frenet(RefPathFrenet, egoState);
end
