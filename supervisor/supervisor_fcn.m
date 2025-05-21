function [M_signal, lane_id, Parking_signal] = supervisor_fcn(Ego_X, Ego_Y)
%#codegen
persistent cur_M_signal cur_lane_id cur_Parking_signal
Lane_id_alpha = 2;
% CASE 1 초기 상태
if isempty(cur_M_signal)
    cur_M_signal = 1;
    cur_lane_id = 2;
    cur_Parking_signal = 0;
end

% ROI 판별 함수 (Axis-Aligned Bounding Box)
in_roi = @(x, y, x_min, x_max, y_min, y_max) ...
    (x >= x_min && x <= x_max && y >= y_min && y <= y_max);

% === ROI별 조건 ===

% CASE 2 M2구간 진입
if in_roi(Ego_X, Ego_Y, -150, -120, -165, -145)
    cur_M_signal = 2;
    cur_lane_id = Lane_id_alpha;
    cur_Parking_signal = 0;

% CASE 3 톨게이트 PASS 후 M3 구간 진입
elseif in_roi(Ego_X, Ego_Y, -178, -162, -72, -60)
    cur_M_signal = 3;
    cur_lane_id = 3;
    cur_Parking_signal = 0;

% CASE 4 M4구간 진입 && M_signal == 3
elseif in_roi(Ego_X, Ego_Y, -50.5, -48.7, 15.5, 25.5) && cur_M_signal == 3
    cur_M_signal = 4;
    cur_lane_id = 4;
    cur_Parking_signal = 0;

% CASE 5 M5구간 진입 && M_signal == 4
elseif in_roi(Ego_X, Ego_Y, -10, -8.5, 11.5, 25.5) && cur_M_signal == 4
    cur_M_signal = 5;
    cur_lane_id = 5;
    cur_Parking_signal = 0;

% CASE 6 M6 : 차량 주차시작
elseif in_roi(Ego_X, Ego_Y, 7.15, 9.5, -39, -34) && cur_M_signal == 5
    cur_Parking_signal = 1;
end

% 출력
M_signal = cur_M_signal;
lane_id = cur_lane_id;
Parking_signal = cur_Parking_signal;
end
