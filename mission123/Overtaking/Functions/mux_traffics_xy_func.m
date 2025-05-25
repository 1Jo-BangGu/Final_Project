function obs_xy = mux_traffics_xy_func(traffic_info)
    data = traffic_info(106:185);
    obs_xy = zeros(8, 2);
    for i = 1:8
        obs_xy(i, 1) = data((i-1)*10+1);
        obs_xy(i, 2) = data((i-1)*10+2);
    end
end