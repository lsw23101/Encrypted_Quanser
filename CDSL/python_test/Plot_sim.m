%% 1. 초기화 및 데이터 로드
clear; clc; close all;

% CSV 파일 읽기 (헤더가 있는 경우 readtable이 편리합니다)
filename = 'plant_data_python.csv';

if ~isfile(filename)
    error('파일을 찾을 수 없습니다: %s \n파이썬 시뮬레이션을 먼저 실행해주세요.', filename);
end

data = readtable(filename);

% 데이터 추출
iter = data.iter;
y0 = data.y0;
y1 = data.y1;
u0 = data.u0;

%% 2. 그래프 그리기
figure('Color', 'w', 'Position', [100, 100, 800, 600]);

% --- (1) System Outputs (y0, y1) ---
subplot(2, 1, 1);
plot(iter, y0, 'LineWidth', 2, 'DisplayName', 'y_0 (Position)');
hold on;
plot(iter, y1, '--', 'LineWidth', 2, 'DisplayName', 'y_1 (Angle)');
yline(0, 'k-', 'HandleVisibility', 'off'); % 0점 기준선
grid on; grid minor;
legend('Location', 'best', 'FontSize', 11);
title('Plant Outputs (y)', 'FontSize', 14);
ylabel('Magnitude');
xlim([0, max(iter)]);

% --- (2) Control Input (u0) ---
subplot(2, 1, 2);
plot(iter, u0, 'r', 'LineWidth', 1.5);
yline(0, 'k-'); % 0점 기준선
grid on; grid minor;
title('Control Input (u)', 'FontSize', 14);
xlabel('Iteration', 'FontSize', 12);
ylabel('Input Value');
xlim([0, max(iter)]);

% 전체 제목
sgtitle('FHE Control Simulation Result (SIMO)', 'FontSize', 16, 'FontWeight', 'bold');

fprintf('>> 그래프 작성 완료.\n');