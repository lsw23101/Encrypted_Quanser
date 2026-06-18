clear; clc; close all;

filename = 'plant_data_python.csv';

data = readtable(filename);

iter = data.iter;
y0 = data.y0;
y1 = data.y1;
u0 = data.u0;

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

sgtitle('FHE Control Simulation Result (SIMO)', 'FontSize', 16, 'FontWeight', 'bold');
