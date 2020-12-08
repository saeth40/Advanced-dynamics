%% Three-Link Planar Robot Manipulator Control

clc;
clear all;
close all;

%% Parameters

params = [];
params.l1       = 1;        % Length of link 1 [m]
params.l2       = 1;        % Length of link 2 [m]
params.l3       = 0;        % Length of link 3 [m]
params.lcm1     = 0.5;      % CM position of link 1 [m]
params.lcm2     = 0.5;      % CM position of link 2 [m]
params.lcm3     = 0.5*0;      % CM position of link 3 [m]
params.m1       = 50;       % Mass of link 1 [kg]
params.m2       = 50;       % Mass of link 2 [kg]
params.m3       = 50*0;       % Mass of link 2 [kg]
params.I_lcm1   = 10;       % Moment of inertia of link 1 [kg.m^2]
params.I_lcm2   = 10;       % Moment of inertia of link 2 [kg.m^2
params.I_lcm3   = 10;       % Moment of inertia of link 3 [kg.m^2]]
params.kr1      = 100*0;      % Gear ratio of motor at joint 1 [-]
params.kr2      = 100*0;      % Gear ratio of motor at joint 2 [-]
params.kr3      = 100*0;      % Gear ratio of motor at joint 3 [-]
params.m_motor1 = 5*0;        % Mass of rotor at joint 1 [kg]
params.m_motor2 = 5*0;        % Mass of rotor at joint 2 [kg]
params.m_motor3 = 5*0;        % Mass of rotor at joint 3 [kg]
params.I_motor1 = 0.01*0;     % Moment of inertia of rotor at joing 1 [kg.m^2]
params.I_motor2 = 0.01*0;     % Moment of inertia of rotor at joing 2 [kg.m^2]
params.I_motor3 = 0.01*0;     % Moment of inertia of rotor at joing 3 [kg.m^2]
params.g        = 9.81;     % Gravity [m/s^2]

%% Setup Kinematics

Ts = 10^-3;
tmax = 20;
tspan = [0:Ts:tmax]';
n = length(tspan);

%% Desired Trajectory 

r = 0.5;
Xd = r*cos(tspan/10)+0.05*tspan;
Yd = r*sin(tspan/10);
Zd = tspan/30;

% Inverse kinematics
[theta1_d,theta2_d,theta3_d] = inverseki(Xd,Yd,Zd,params);
%Foward kinematics
[Xdd,Ydd,Zdd] = forwardki(theta1_d,theta2_d,theta3_d,params);

%% Plot Kinematics Profiles

% plot profile
figure;
subplot(3,5,[1,2,3,6,7,8,11,12,13])
set(gcf, 'Position', [350 100 2000 1200]/2);
plot3(Xd,Yd,Zd,'LineWidth', 2, 'Color', 'g');
hold on
plot3(Xdd,Ydd,Zdd,'LineWidth', 2,'LineStyle', '--', 'Color', 'k');
legend('Desired Path','Calculated Path')
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]')
set(gca, 'FontSize', 12); grid on;title('Desired Profiles');
view(-10,10)
grid on;
% plot position
subplot(3,5,[4,5])
plot(tspan,Xd, 'LineWidth', 2, 'Color', 'y');
hold on
plot(tspan,Xdd, 'LineWidth', 2,'LineStyle', '--', 'Color', 'r');
xlabel('Time [s]'); ylabel('X [m]');
set(gca, 'FontSize', 8); grid on;title('X position');
legend('Desired Path','Calculated Path')

subplot(3,5,[9,10])
plot(tspan,Yd, 'LineWidth', 2, 'Color', 'y'); 
hold on
plot(tspan,Ydd, 'LineWidth', 2,'LineStyle', '--', 'Color', 'r');
xlabel('Time [s]'); ylabel('Y [m]');
set(gca, 'FontSize', 8); grid on;title('Y position');
legend('Desired Path','Calculated Path')

subplot(3,5,[14,15])
plot(tspan,Zd, 'LineWidth', 2, 'Color', 'y'); 
hold on
plot(tspan,Zdd, 'LineWidth', 2,'LineStyle', '--', 'Color', 'r');
xlabel('Time [s]'); ylabel('Z [m]');
set(gca, 'FontSize', 8); grid on;title('Z position');
legend('Desired Path','Calculated Path')

% plot theta
figure;
set(gcf, 'Position', [350 100 1500 1200]/2);
subplot(3,1,1)
plot(tspan,theta1_d*180/pi, 'LineWidth', 2, 'Color', 'r'); xlabel('Time [s]'); ylabel('theta1 [deg]');
set(gca, 'FontSize', 10); grid on;title('Theta1');
subplot(3,1,2)
plot(tspan,theta2_d*180/pi, 'LineWidth', 2, 'Color', 'b'); xlabel('Time [s]'); ylabel('theta2 [deg]');
set(gca, 'FontSize', 10); grid on;title('Theta2');
subplot(3,1,3)
plot(tspan,theta3_d*180/pi, 'LineWidth', 2, 'Color', 'g'); xlabel('Time [s]'); ylabel('theta3 [deg]');
set(gca, 'FontSize', 10); grid on;title('Theta3');

%% Desired Kinematics Profiles (Theta_d, Theta_d_dot, Theta_d_ddot)

% Start at Zero Velocity
theta1_d_dot = [0; diff(theta1_d)/Ts];
theta2_d_dot = [0; diff(theta2_d)/Ts];
theta3_d_dot = [0; diff(theta3_d)/Ts];

% Start at Zero Acceleration
theta1_d_ddot = [0; diff(theta1_d_dot)/Ts]; 
theta2_d_ddot = [0; diff(theta2_d_dot)/Ts];
theta3_d_ddot = [0; diff(theta3_d_dot)/Ts];

Kinematics_Profiles = [];

Kinematics_Profiles.theta1_d      = theta1_d;
Kinematics_Profiles.theta1_d_dot  = theta1_d_dot;
Kinematics_Profiles.theta1_d_ddot = theta1_d_ddot;

Kinematics_Profiles.theta2_d      = theta2_d;
Kinematics_Profiles.theta2_d_dot  = theta2_d_dot;
Kinematics_Profiles.theta2_d_ddot = theta2_d_ddot;

Kinematics_Profiles.theta3_d      = theta3_d;
Kinematics_Profiles.theta3_d_dot  = theta3_d_dot;
Kinematics_Profiles.theta3_d_ddot = theta3_d_ddot;

figure;
set(gcf, 'Position', [350 100 2000 1200]/2);
for ii = 1:1
    
    % Theta1 --------------------------------------------------------------
    
    subplot(3,3,1); plot(tspan, theta1_d*180/pi, 'LineWidth', 2, 'Color', 'b'); 
    xlabel('Time'); ylabel('Theta1d [deg]'); set(gca, 'FontSize', 11);title('Theta1d'); grid on;
    
    subplot(3,3,4); plot(tspan, theta1_d_dot*180/pi, 'LineWidth', 2, 'Color', 'b'); 
    xlabel('Time'); ylabel('Theta1d-dot [deg/s]'); set(gca, 'FontSize', 11);title('Theta1d-dot'); grid on;
    
    subplot(3,3,7); plot(tspan, theta1_d_ddot*180/pi, 'LineWidth', 2, 'Color', 'b'); 
    xlabel('Time'); ylabel('Theta1d-ddot [deg/s^2]'); set(gca, 'FontSize', 11);title('Theta1d-ddot');grid on;ylim([-10 10]);
    
    % Theta2 --------------------------------------------------------------
    subplot(3,3,2); plot(tspan, theta2_d*180/pi, 'LineWidth', 2, 'Color', [0  0.5  0]); 
    xlabel('Time'); ylabel('Theta2d [deg]'); set(gca, 'FontSize', 11);title('Theta2d'); grid on;
    
    subplot(3,3,5); plot(tspan, theta2_d_dot*180/pi, 'LineWidth', 2, 'Color', [0  0.5  0]); 
    xlabel('Time'); ylabel('Theta2d-dot [deg/s]'); set(gca, 'FontSize', 11);title('Theta2d-dot'); grid on;
    
    subplot(3,3,8); plot(tspan, theta2_d_ddot*180/pi, 'LineWidth', 2, 'Color', [0  0.5  0]); 
    xlabel('Time'); ylabel('Theta2d-ddot [deg/s^2]'); set(gca, 'FontSize', 11);title('Theta2d-ddot'); grid on;ylim([-10 10]);  
    
    % Theta3 --------------------------------------------------------------
    subplot(3,3,3); plot(tspan, theta3_d*180/pi, 'LineWidth', 2, 'Color', [1  0  0]); 
    xlabel('Time'); ylabel('Theta3d [deg]'); set(gca, 'FontSize', 11);title('Theta3d');grid on;
    
    subplot(3,3,6); plot(tspan, theta3_d_dot*180/pi, 'LineWidth', 2, 'Color', [1  0  0]); 
    xlabel('Time'); ylabel('Theta3d-dot [deg/s]'); set(gca, 'FontSize', 11);title('Theta3d-dot');grid on;
    
    subplot(3,3,9); plot(tspan, theta3_d_ddot*180/pi, 'LineWidth', 2, 'Color', [1  0  0]); 
    xlabel('Time'); ylabel('Theta3d-ddot [deg/s^2]'); set(gca, 'FontSize', 11);title('Theta3d-ddot');grid on;ylim([-10 10]); 
end

%% Solve EOM (Open Loop)

%initial condition [position:velocity]
X0 = [pi/2;0;0;0;0;0];
   
fun = @(t,X)advdyManipulator(t, X, params);
[tout, Xout] = ode45(fun, tspan, X0, []);

theta1     = Xout(:,1);
theta1_dot = Xout(:,2);
theta2     = Xout(:,3);
theta2_dot = Xout(:,4);
theta3     = Xout(:,5);
theta3_dot = Xout(:,6);

%% Simulation Kinematics Plots

ind_plot = 1;
if(ind_plot)
    
figure;
set(gcf, 'Position', [500 300 2000 1200]/2);
grid on;
axis(2*[-1 1 -1 1 -1 1]);
grid on;
for ii = 1:100:length(theta1) 
    %link 1
     plot3([0 -params.l1*cos(theta1(ii))*sin(theta3(ii))],...
          [0 params.l1*cos(theta1(ii))*cos(theta3(ii))],...
          [0 params.l1*sin(theta1(ii))],...
           'linewidth', 2, 'Color', 'r');        
    hold all;
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    set(gca, 'FontSize', 16);
    view(45,45);
    %motor 2
     plot3(-params.l1*cos(theta1(ii))*sin(theta3(ii)),...
           params.l1*cos(theta1(ii))*cos(theta3(ii)),...
           params.l1*sin(theta1(ii)),...
           'lineStyle', 'None', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
    %link 2 
    plot3([-params.l1*cos(theta1(ii))*sin(theta3(ii)) -(params.l1*cos(theta1(ii))+params.l2*cos(theta1(ii)+theta2(ii)))*sin(theta3(ii))],...
          [params.l1*cos(theta1(ii))*cos(theta3(ii)) (params.l1*cos(theta1(ii))+params.l2*cos(theta1(ii)+theta2(ii)))*cos(theta3(ii))],...
          [params.l1*sin(theta1(ii)) params.l1*sin(theta1(ii))+params.l2*sin(theta1(ii)+theta2(ii))],...
        'linewidth', 2, 'Color', 'g');
    %End effector
    plot3(-(params.l1*cos(theta1(ii))+params.l2*cos(theta1(ii)+theta2(ii)))*sin(theta3(ii)),...
          (params.l1*cos(theta1(ii))+params.l2*cos(theta1(ii)+theta2(ii)))*cos(theta3(ii)),...
           params.l1*sin(theta1(ii))+params.l2*sin(theta1(ii)+theta2(ii)),...
           'lineStyle', 'None', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor','g', 'MarkerEdgeColor', 'g');
    %Axis
    plot3([-2 2],[0 0],[0 0],'LineWidth', 2, 'Color', 'k');
    plot3([0 0],[-2 2],[0 0],'LineWidth', 2, 'Color', 'k');
    plot3([0 0],[0 0] ,[-2 2],'LineWidth', 2, 'Color', 'k');
    pause( 0.1 );
    if (ii~= length(theta1))
        clf;
    else
        % Do nothing, Do not clear figure if not final post
    end
    
end

end % if(ind_plot)

%% Closed Loop EOM

X0 = [0;0;0;0;0;0];
   
fun_fb = @(t,X)advdyfeedback(t, X, tspan, params, Xd, Yd, Kinematics_Profiles);
[tout_fb, Xout_fb] = ode45(fun_fb, tspan, X0, []);

theta1_fb     = Xout_fb(:,1);
theta1_dot_fb = Xout_fb(:,2);
theta2_fb     = Xout_fb(:,3);
theta2_dot_fb = Xout_fb(:,4);
theta3_fb     = Xout_fb(:,5);
theta3_dot_fb = Xout_fb(:,6);

l1 = params.l1;
l2 = params.l2;
l3 = params.l3;

[X_fb,Y_fb,Z_fb] = forwardki(theta1_fb,theta2_fb,theta3_fb,params);

%% Plot closed-loop profile

% X-Y-Z destired vs control
figure;
set(gcf, 'Position', [350 100 2000 1200]/2);
subplot(3,5,[1,2,3,6,7,8,11,12,13])
plot3(Xd, Yd, Zd,'LineWidth', 2, 'Color', 'm'); 
hold on;
plot3(X_fb, Y_fb,Z_fb, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k'); 
grid on
view(-15,10);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]')
set(gca, 'FontSize', 12); grid on;title('Kinematics Profiles');
legend('Desired','Control')

% X destired vs control
subplot(3,5,[4,5])
plot(tspan,Xd, 'LineWidth', 2, 'Color', 'r'); xlabel('Time [s]'); ylabel('X [m]');
hold on
plot(tspan,X_fb, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'k'); 
set(gca, 'FontSize', 8); grid on;title('X position');
legend('Desired','Control')

% Y destired vs control
subplot(3,5,[9,10])
plot(tspan,Yd, 'LineWidth', 2, 'Color', 'g'); xlabel('Time [s]'); ylabel('Y [m]');
hold on
plot(tspan,Y_fb, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'k'); 
set(gca, 'FontSize', 8); grid on;title('Y position');
legend('Desired','Control')

% Z destired vs control
subplot(3,5,[14,15])
plot(tspan,Zd, 'LineWidth', 2, 'Color', 'c'); xlabel('Time [s]'); ylabel('Z [m]');
hold on
plot(tspan,Z_fb, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'k'); 
set(gca, 'FontSize', 8); grid on;title('Z position');
legend('Desired','Control')

% Theta destired vs control
figure;
set(gcf, 'Position', [350 100 1500 1200]/2);
legend('Desired','Control')

% theta1 destired vs control
subplot(3,1,1)
plot(tspan,theta1_d*180/pi, 'LineWidth', 2, 'Color', 'r'); xlabel('Time [s]'); ylabel('Theta1 [deg]');
hold on
plot(tspan,theta1_fb*180/pi, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'k'); 
set(gca, 'FontSize', 12); grid on;title('Theta1');
legend('Desired','Control')

% theta2 destired vs control
subplot(3,1,2)
plot(tspan,theta2_d*180/pi, 'LineWidth', 2, 'Color', 'g'); xlabel('Time [s]'); ylabel('Theta2 [deg]');
hold on
plot(tspan,theta2_fb*180/pi, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'k'); 
set(gca, 'FontSize', 12); grid on;title('Theta2');
legend('Desired','Control')

% theta3 destired vs control
subplot(3,1,3)
plot(tspan,theta3_d*180/pi, 'LineWidth', 2, 'Color', 'c'); xlabel('Time [s]'); ylabel('Theta3 [deg]');
hold on
plot(tspan,theta3_fb*180/pi, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'k'); 
set(gca, 'FontSize', 12); grid on;title('Theta3');
legend('Desired','Control')

%% Simulation+Control

ind_plot = 1;
if(ind_plot)
    
figure;
set(gcf, 'Position', [500 300 2000 1200]/2);
grid on;
axis(2*[-1 1 -1 1 -1 1]);


for ii = 1:100:length(theta1_fb) 
    plot3(Xd,Yd,Zd,'linewidth', 3, 'Color', 'b');
    hold all;
    plot3(X_fb, Y_fb,Z_fb, 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'k'); 
    %link 1
     plot3([0 -params.l1*cos(theta1_fb(ii))*sin(theta3_fb(ii))],...
          [0 params.l1*cos(theta1_fb(ii))*cos(theta3_fb(ii))],...
          [0 params.l1*sin(theta1_fb(ii))],...
           'linewidth', 2, 'Color', 'r');   
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    set(gca, 'FontSize', 16);

    axis(1.5*[-1 1 -1 1 -1 1]);
    view(-5,40);
    %motor 2
     plot3(-params.l1*cos(theta1_fb(ii))*sin(theta3_fb(ii)),...
           params.l1*cos(theta1_fb(ii))*cos(theta3_fb(ii)),...
           params.l1*sin(theta1_fb(ii)),...
           'lineStyle', 'None', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');
    %link 2 
    plot3([-params.l1*cos(theta1_fb(ii))*sin(theta3_fb(ii)) -(params.l1*cos(theta1_fb(ii))+params.l2*cos(theta1_fb(ii)+theta2_fb(ii)))*sin(theta3_fb(ii))],...
          [params.l1*cos(theta1_fb(ii))*cos(theta3_fb(ii)) (params.l1*cos(theta1_fb(ii))+params.l2*cos(theta1_fb(ii)+theta2_fb(ii)))*cos(theta3_fb(ii))],...
          [params.l1*sin(theta1_fb(ii)) params.l1*sin(theta1_fb(ii))+params.l2*sin(theta1_fb(ii)+theta2_fb(ii))],...
        'linewidth', 2, 'Color', 'g');
    %End effector
    plot3(-(params.l1*cos(theta1_fb(ii))+params.l2*cos(theta1_fb(ii)+theta2_fb(ii)))*sin(theta3_fb(ii)),...
          (params.l1*cos(theta1_fb(ii))+params.l2*cos(theta1_fb(ii)+theta2_fb(ii)))*cos(theta3_fb(ii)),...
           params.l1*sin(theta1_fb(ii))+params.l2*sin(theta1_fb(ii)+theta2_fb(ii)),...
           'lineStyle', 'None', 'Marker', 'o', 'MarkerSize', 15, 'MarkerFaceColor','g', 'MarkerEdgeColor', 'g');
    pause( 0.1 );
    if (ii~= length(theta1_fb))
        clf;
    else
        % Do nothing, Do not clear figure if not final post
    end
    
end

end % if(ind_plot)

%% Simmechanic Free fall

sim('Scalar3DOF_final');
figure;
set(gcf, 'Position', [350 100 1500 1200]/2);
% X 
subplot(3,1,1)
plot(tspan,theta1*180/pi, 'LineWidth', 2, 'Color', 'r'); xlabel('Time [s]'); ylabel('Theta1 [deg]');
hold on
plot(theta1ans.time,theta1ans.data*180/pi, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'k'); 
set(gca, 'FontSize', 10); grid on; title('Theta1');
legend('MATLAB','Simulink');

% Y 
subplot(3,1,2)
plot(tspan,theta2*180/pi, 'LineWidth', 2, 'Color', 'g'); xlabel('Time [s]'); ylabel('Theta2 [deg]');
hold on
plot(theta2ans.time,theta2ans.data*180/pi, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'k'); 
set(gca, 'FontSize', 10); grid on; title('Theta2');
legend('MATLAB','Simulink');

% Z 
subplot(3,1,3)
plot(tspan,theta3*180/pi, 'LineWidth', 2, 'Color', 'c'); xlabel('Time [s]'); ylabel('Theta3 [deg]');
hold on
plot(theta3ans.time,theta3ans.data*180/pi, 'LineWidth', 2, 'LineStyle', '--', 'Color', 'k'); 
set(gca, 'FontSize', 10); grid on; title('Theta3');ylim([-1,1]);
legend('MATLAB','Simulink');

%% Simmechanic Control
theta1inp = [tspan theta1_fb];
theta2inp = [tspan theta2_fb];
theta3inp = [tspan theta3_fb];
sim('Scalar3DOF_control_final');
%% Simmechanic Control

sim('Scalar3DOF_control_final');
figure;
set(gcf, 'Position', [350 100 1500 1200]/2);
% X 
subplot(3,1,1)
plot(torque1.time,torque1.data, 'LineWidth', 2, 'Color', 'r'); xlabel('Time [s]'); ylabel('Torque 1');

% Y 
subplot(3,1,2)
plot(torque2.time,torque2.data, 'LineWidth', 2, 'Color', 'g'); xlabel('Time [s]'); ylabel('Torque 2');

% Z 
subplot(3,1,3)
plot(torque3.time,torque3.data, 'LineWidth', 2, 'Color', 'c'); xlabel('Time [s]'); ylabel('Torque 3');
