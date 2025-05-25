%% MATLAB Script 코드 (.m 파일)

function coeffs = quintic_curve_generate(s0, d0, ds0, dds0, s1, d1)
    % quintic 경로: d(s) = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5
    % 조건: 위치, 속도, 가속도 (초기/최종) 총 6개

    % 초기 조건
    L = s1 - s0;
    if L == 0
        coeffs = zeros(1,6);
        return;
    end
   
    ds0 = ds0 / L;
    dds0 = dds0 / L;
    
    % 최종 조건
    ds1 = ds0;   % 종단 속도
    dds1 = dds0;  % 종단 가속도

    % 행렬 방정식 설정
    A = [1 s0 s0^2 s0^3 s0^4 s0^5;
         0 1  2*s0 3*s0^2 4*s0^3 5*s0^4;
         0 0  2    6*s0   12*s0^2 20*s0^3;
         1 s1 s1^2 s1^3 s1^4 s1^5;
         0 1  2*s1 3*s1^2 4*s1^3 5*s1^4;
         0 0  2    6*s1   12*s1^2 20*s1^3];

    b = [d0; ds0; dds0; d1; ds1; dds1];

    % 해 구하기
    coeffs = A \ b;
end