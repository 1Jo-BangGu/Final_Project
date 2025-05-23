E_X = E_X.signals.values;     % 실제 데이터 추출
save('E_X.mat', 'E_X');  % 파일 저장
E_Y = E_Y.signals.values;     % 실제 데이터 추출
save('E_Y.mat', 'E_Y');  % 파일 저장
oc = oc.signals.values;     % 실제 데이터 추출
save('oc.mat', 'oc');  % 파일 저장

save('route1_waypoint.mat', 'route1_waypoint');  % 파일 저장

save('route2_waypoint.mat', 'route2_waypoint');  % 파일 저장

save('route3_waypoint.mat', 'route3_waypoint');  % 파일 저장
candidate_oc = candidate_oc.signals.values;     % 실제 데이터 추출
save('candidate_oc.mat', 'candidate_oc');  % 파일 저장

obstacle = obstacle.signals.values;     % 실제 데이터 추출
save('obstacle.mat', 'obstacle');  % 파일 저장
