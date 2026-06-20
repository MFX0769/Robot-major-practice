%% 4轴机械臂 几何法IK + 可视化 (纯MATLAB, 无toolbox依赖)
% 万一 Robotics Toolbox 没 license, 这个也能跑

clear; clc; close all;

%% 机械臂参数 (跟 C 代码保持一致)
L1 = 0.03;   % 底座高度 (m)
L2 = 0.15;   % 大臂
L3 = 0.15;   % 小臂
L4 = 0.12;   % 夹爪

%% 几何法 IK
target = [0.15; 0.15; 0.20];  % 目标位置 x y z (前方15cm, 侧方15cm, 高20cm)
alpha = 0;                     % 夹爪俯仰 (水平向前)

x = target(1); y = target(2); z = target(3);

t1 = atan2(y, x);
r  = sqrt(x^2 + y^2);
zp = z - L1;
rw = r  - L4 * cos(alpha);
zw = zp - L4 * sin(alpha);
d  = sqrt(rw^2 + zw^2);

if d > L2 + L3 || d < abs(L2 - L3)
    error('目标不可达 d=%.3f', d);
end

c3 = (d^2 - L2^2 - L3^2) / (2*L2*L3);
c3 = max(-1, min(1, c3));
t3 = acos(c3);
t2 = atan2(rw, zw) - atan2(L3*sin(t3), L2 + L3*cos(t3));
t4 = (pi/2 + alpha) - (t2 + t3);

q_ik = [t1; t2; t3; t4];
fprintf('IK结果: θ1=%.1f°  θ2=%.1f°  θ3=%.1f°  θ4=%.1f°\n', rad2deg(q_ik));

%% 正运动学验证
T = eye(4);

% J1
T1 = T * rotz(t1) * transl(0,0,L1);
% J2
T2 = T1 * roty(t2) * transl(0,0,L2);
% J3
T3 = T2 * roty(t3) * transl(0,0,L3);
% J4 + 夹爪
T4 = T3 * roty(t4) * transl(0,0,L4);

pos = [T(1:3,4)'; T1(1:3,4)'; T2(1:3,4)'; T3(1:3,4)'; T4(1:3,4)'];
fprintf('末端位置: (%.3f, %.3f, %.3f)\n', T4(1,4), T4(2,4), T4(3,4));
err = norm(T4(1:3,4) - target);
fprintf('位置误差: %.1f mm\n', err*1000);

%% 可视化
figure('Name','几何IK验证', 'NumberTitle','off', 'Position',[100 100 800 600]);
plot3(pos(:,1), pos(:,2), pos(:,3), 'o-', 'LineWidth', 3, 'MarkerSize', 8, ...
    'MarkerFaceColor', 'b');
hold on;
plot3(target(1), target(2), target(3), 'r*', 'MarkerSize', 20, 'LineWidth', 2);
plot3(0,0,0, 'ks', 'MarkerSize', 10, 'LineWidth', 2);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45, 25);
title(sprintf('几何法IK验证  误差: %.1f mm', err*1000));
legend({'机械臂连杆','目标位置','基座'}, 'Location','best');

% 关节标签
labels = {'base','J1 yaw','J2 shoulder','J3 elbow','J4 gripper'};
for i = 1:5
    text(pos(i,1), pos(i,2), pos(i,3)+0.01, labels{i}, 'FontSize', 10);
end

%% 辅助函数 (纯MATLAB, 无toolbox)
function T = rotz(a)
    ca = cos(a); sa = sin(a);
    T = [ca -sa 0 0; sa ca 0 0; 0 0 1 0; 0 0 0 1];
end

function T = roty(b)
    cb = cos(b); sb = sin(b);
    T = [cb 0 sb 0; 0 1 0 0; -sb 0 cb 0; 0 0 0 1];
end

function T = transl(dx,dy,dz)
    T = [1 0 0 dx; 0 1 0 dy; 0 0 1 dz; 0 0 0 1];
end
