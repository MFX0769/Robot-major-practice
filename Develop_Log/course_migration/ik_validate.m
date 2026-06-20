%% 4轴机械臂 IK 验证 (Yaw + 肩 + 肘 + 夹爪)
% 臂长: 大臂150mm 小臂150mm 末端(夹爪)120mm

clear; clc; close all;

%% 1. 建立机械臂模型 (rigidBodyTree)
robot = rigidBodyTree('DataFormat','column');

% --- 基座 -> Yaw (绕Z轴旋转) ---
body1 = rigidBody('yaw');
jnt1 = rigidBodyJoint('jnt_yaw', 'revolute');
jnt1.HomePosition = 0;
jnt1.JointAxis = [0 0 1];  % 绕Z
setFixedTransform(jnt1, trvec2tform([0 0 0]));
body1.Joint = jnt1;
addBody(robot, body1, 'base');

% --- Yaw -> Shoulder (绕Y轴旋转, 连杆沿Z) ---
body2 = rigidBody('shoulder');
jnt2 = rigidBodyJoint('jnt_shoulder', 'revolute');
jnt2.HomePosition = 0;
jnt2.JointAxis = [0 1 0];  % 绕Y
jnt2.PositionLimits = [-pi/2, pi/2];
setFixedTransform(jnt2, trvec2tform([0 0 0.03]));  % 底座高度3cm
body2.Joint = jnt2;
addBody(robot, body2, 'yaw');

% --- Shoulder -> Elbow (大臂 150mm, 绕Y) ---
body3 = rigidBody('elbow');
jnt3 = rigidBodyJoint('jnt_elbow', 'revolute');
jnt3.HomePosition = 0;
jnt3.JointAxis = [0 1 0];  % 绕Y
jnt3.PositionLimits = [-pi, pi];
setFixedTransform(jnt3, trvec2tform([0 0 0.15]));  % 大臂150mm
body3.Joint = jnt3;
addBody(robot, body3, 'shoulder');

% --- Elbow -> Gripper (小臂 150mm, 绕Y) ---
body4 = rigidBody('gripper');
jnt4 = rigidBodyJoint('jnt_gripper', 'revolute');
jnt4.HomePosition = 0;
jnt4.JointAxis = [0 1 0];  % 绕Y
jnt4.PositionLimits = [-pi/2, pi/2];
setFixedTransform(jnt4, trvec2tform([0 0 0.15]));  % 小臂150mm
body4.Joint = jnt4;
addBody(robot, body4, 'elbow');

% --- Gripper -> End Effector (夹爪 120mm, 固定关节) ---
body5 = rigidBody('endeffector');
setFixedTransform(body5.Joint, trvec2tform([0 0 0.12]));  % 夹爪120mm
addBody(robot, body5, 'gripper');

%% 2. 测试正运动学
q_test = [0.3; -0.5; 1.0; -0.3];
tform = getTransform(robot, q_test, 'endeffector');
pos = tform(1:3, 4)';
fprintf('正运动学测试 - 末端位置: x=%.3f  y=%.3f  z=%.3f (m)\n', pos);

%% 3. IK
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations = 500;
weights = [0 0 0 1 1 1];  % 只约束位置, 不约束姿态
q_init = [0.5; -0.3; 1.0; -0.2];

target_pos = [0.15; 0.15; 0.20];
target_tform = trvec2tform(target_pos');

[q_sol, info] = ik('endeffector', target_tform, weights, q_init);

tform_sol = getTransform(robot, q_sol, 'endeffector');
pos_sol = tform_sol(1:3, 4)';
err = norm(pos_sol - target_pos');
fprintf('IK求解: 目标(%.3f,%.3f,%.3f) -> 到达(%.3f,%.3f,%.3f) 误差%.3fmm\n', ...
    target_pos(1),target_pos(2),target_pos(3), pos_sol(1),pos_sol(2),pos_sol(3), err*1000);

fprintf('关节角: θ1=%.1f°  θ2=%.1f°  θ3=%.1f°  θ4=%.1f°\n', ...
    rad2deg(q_sol(1)), rad2deg(q_sol(2)), rad2deg(q_sol(3)), rad2deg(q_sol(4)));

%% 4. 可视化
figure('Name','4轴机械臂 IK验证', 'NumberTitle','off', 'Position',[100 100 900 700]);
show(robot, q_sol, 'Frames','on', 'PreservePlot',true);
hold on;
plot3(target_pos(1), target_pos(2), target_pos(3), 'r*', 'MarkerSize', 20, 'LineWidth', 2);
grid on;
axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45, 30);
xlim([-0.1 0.4]); ylim([-0.2 0.3]); zlim([0 0.5]);
title(sprintf('4轴机械臂 IK验证  误差: %.1f mm', err*1000));
legend({'模型','坐标系','目标点'}, 'Location','best');
drawnow;

%% 5. 验证几何IK一致性
L1=0.03; L2=0.15; L3=0.15; L4=0.12;
alpha = 0;
x=target_pos(1); y=target_pos(2); z=target_pos(3);
t1 = atan2(y, x);
r = sqrt(x^2 + y^2);
zp = z - L1;
rw = r - L4*cos(alpha);
zw = zp - L4*sin(alpha);
d = sqrt(rw^2 + zw^2);
c3 = (d^2 - L2^2 - L3^2) / (2*L2*L3);
c3 = max(-1, min(1, c3));
t3 = acos(c3);
t2 = atan2(rw, zw) - atan2(L3*sin(t3), L2 + L3*cos(t3));
t4 = (pi/2 + alpha) - (t2 + t3);
q_geo = [t1; t2; t3; t4];
fprintf('\n几何IK vs Robotics Toolbox IK:\n');
fprintf('  几何法: θ1=%.1f° θ2=%.1f° θ3=%.1f° θ4=%.1f°\n', rad2deg(q_geo));
fprintf('  Toolbox: θ1=%.1f° θ2=%.1f° θ3=%.1f° θ4=%.1f°\n', rad2deg(q_sol));
fprintf('  角度差: %.1f°\n', rad2deg(norm(q_geo - q_sol)));

%% 6. 保存结果
save('ik_validate_result.mat', 'robot', 'q_sol', 'q_geo', 'target_pos', 'pos_sol', 'err');
fprintf('\n结果已保存到 ik_validate_result.mat\n');
