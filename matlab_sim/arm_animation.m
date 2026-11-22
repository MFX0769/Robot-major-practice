function arm_animation()
clear; close all;
L1=0.03; L2=0.15; L3=0.15; L4=0.12;
alpha=0; R=0.018;

%% 建立机器人模型 (带可视化几何体)
robot=rigidBodyTree('DataFormat','column');
% Yaw
b1=rigidBody('yaw'); j1=rigidBodyJoint('jnt_yaw','revolute');
j1.JointAxis=[0 0 1]; setFixedTransform(j1,trvec2tform([0 0 0]));
b1.Joint=j1; addVisual(b1,'Cylinder',[0.025 0.02]); addBody(robot,b1,'base');
% Shoulder
b2=rigidBody('shoulder'); j2=rigidBodyJoint('jnt_shoulder','revolute');
j2.JointAxis=[0 1 0]; setFixedTransform(j2,trvec2tform([0 0 L1]));
b2.Joint=j2; addVisual(b2,'Cylinder',[R L2]); addBody(robot,b2,'yaw');
% Elbow
b3=rigidBody('elbow'); j3=rigidBodyJoint('jnt_elbow','revolute');
j3.JointAxis=[0 1 0]; setFixedTransform(j3,trvec2tform([0 0 0]));
b3.Joint=j3; addVisual(b3,'Cylinder',[R L3]); addBody(robot,b3,'shoulder');
% Gripper base
b4=rigidBody('gripper'); j4=rigidBodyJoint('jnt_gripper','revolute');
j4.JointAxis=[0 1 0]; setFixedTransform(j4,trvec2tform([0 0 0]));
b4.Joint=j4; addVisual(b4,'Box',[0.03 0.03 0.06]); addBody(robot,b4,'elbow');
% End
b5=rigidBody('endeffector'); setFixedTransform(b5.Joint,trvec2tform([0 0 L3+L4]));
addVisual(b5,'Cylinder',[R*0.6 0.04]); addBody(robot,b5,'gripper');

%% 预计算轨迹
total_duration=20; fps=30; n_frames=total_duration*fps;
t=linspace(0,total_duration,n_frames)';
q=zeros(n_frames,4); pos=zeros(n_frames,3); tgt=zeros(n_frames,3);
fprintf('计算 %d 帧...\n',n_frames);
for i=1:n_frames
    tt=t(i); y=0.5*sin(tt*0.8); r=0.22+0.06*sin(tt*0.5); h=0.15+0.05*sin(tt*0.6);
    target=[r*cos(y);r*sin(y);h];
    qq=ik(L1,L2,L3,L4,target,alpha); q(i,:)=qq';
    p=fk(L1,L2,L3,L4,qq); pos(i,:)=p; tgt(i,:)=target';
end

%% 显示
opengl hardware;
fig=figure('Name','4轴机械臂 运动仿真','NumberTitle','off',...
    'Position',[50 50 1100 800],'Color',[0.12 0.12 0.15]);
ax=axes('Parent',fig,'Position',[0.05 0.1 0.72 0.82],...
    'Color',[0.12 0.12 0.15],'XColor',[0.6 0.6 0.6],...
    'YColor',[0.6 0.6 0.6],'ZColor',[0.6 0.6 0.6]);
hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
xlim(ax,[-0.45 0.55]); ylim(ax,[-0.45 0.45]); zlim(ax,[0 0.6]);
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
view(ax,45,25); title(ax,'4轴机械臂 运动仿真','Color','w','FontSize',14);

% 地面
[gx,gy]=meshgrid(-0.6:0.1:0.6,-0.6:0.1:0.6);
surf(ax,gx,gy,zeros(size(gx)),'FaceColor',[0.18 0.18 0.2],...
    'EdgeColor',[0.25 0.25 0.3],'FaceAlpha',0.6,'EdgeAlpha',0.2);
camlight(ax,'headlight'); lighting gouraud;

% 初始显示
q0=q(1,:)';
h_show=show(robot,q0,'Parent',ax,'PreservePlot',true,'Frames','off');
drawnow;

% 目标点 + 轨迹
h_tgt=scatter3(ax,tgt(1,1),tgt(1,2),tgt(1,3),200,'r*','LineWidth',2);
h_traj=plot3(ax,nan,nan,nan,'-','LineWidth',1.5,'Color',[0.3 0.8 1 0.4]);

% 信息面板
info=uicontrol('Style','text','Position',[790 40 280 770],...
    'BackgroundColor',[0.14 0.14 0.18],'ForegroundColor',[0.9 0.9 0.9],...
    'FontSize',12,'HorizontalAlignment','left','FontName','Consolas');

%% 动画
fprintf('播放中 (2x加速)...\n'); N=round(n_frames/2);
for i=1:2:N
    if ~ishghandle(fig), break; end
    show(robot,q(i,:)','Parent',ax,'PreservePlot',false,'Frames','off','FastUpdate',true);
    set(h_tgt,'XData',tgt(i,1),'YData',tgt(i,2),'ZData',tgt(i,3));
    set(h_traj,'XData',pos(1:i,1),'YData',pos(1:i,2),'ZData',pos(1:i,3));
    err=norm(pos(i,:)-tgt(i,:))*1000;
    str=sprintf([' ⏱  %.1f s\n' ...
        '━━━━━━━━━━━━━━━━\n' ...
        '  关 节 角\n' ...
        '  Yaw      %6.1f°\n' ...
        '  Shoulder %6.1f°\n' ...
        '  Elbow    %6.1f°\n' ...
        '  Gripper  %6.1f°\n' ...
        '━━━━━━━━━━━━━━━━\n' ...
        '  末端  (%d, %d, %d) mm\n' ...
        '  目标  (%d, %d, %d) mm\n' ...
        '━━━━━━━━━━━━━━━━\n' ...
        '  ✕  误 差  %.1f mm'],...
        t(i),rad2deg(q(i,:)),round(pos(i,:)*1000),round(tgt(i,:)*1000),err);
    set(info,'String',str);
    drawnow('limitrate');
end
fprintf('完成!\n');
end

function q=ik(L1,L2,L3,L4,target,a)
    x=target(1);y=target(2);z=target(3);
    t1=atan2(y,x); r=sqrt(x^2+y^2);
    rw=r-L4*cos(a); zw=z-L1-L4*sin(a); d=sqrt(rw^2+zw^2);
    if d>L2+L3||d<abs(L2-L3), q=[t1;pi/4;-pi/2;0]; return; end
    c3=(d^2-L2^2-L3^2)/(2*L2*L3); c3=max(-1,min(1,c3)); t3=acos(c3);
    t2=atan2(rw,zw)-atan2(L3*sin(t3),L2+L3*cos(t3));
    t4=pi/2+a-(t2+t3); q=[t1;t2;t3;t4];
end
function p=fk(L1,L2,L3,L4,q)
    T=eye(4); T=T*rotz(q(1))*transl(0,0,0);
    T=T*roty(q(2))*transl(0,0,L1);
    T=T*roty(q(3))*transl(0,0,L2);
    T=T*roty(q(4))*transl(0,0,L3);
    T=T*transl(0,0,L4); p=T(1:3,4)';
end
function T=rotz(a)
    ca=cos(a);sa=sin(a);T=[ca -sa 0 0;sa ca 0 0;0 0 1 0;0 0 0 1];
end
function T=roty(b)
    cb=cos(b);sb=sin(b);T=[cb 0 sb 0;0 1 0 0;-sb 0 cb 0;0 0 0 1];
end
function T=transl(dx,dy,dz)
    T=[1 0 0 dx;0 1 0 dy;0 0 1 dz;0 0 0 1];
end
