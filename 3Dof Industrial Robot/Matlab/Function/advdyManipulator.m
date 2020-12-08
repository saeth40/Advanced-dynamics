function X_dot = Neom_3DOF_planar_robot_manipulator(t, X, params)

% State -------------------------------------------------------------------

theta1      = X(1);
theta1_dot  = X(2);
theta2      = X(3);
theta2_dot  = X(4);
theta3      = X(5);
theta3_dot  = X(6);

% Parameters --------------------------------------------------------------
params;
l1          = params.l1;            % Length of link 1 [m]
l2          = params.l2;            % Length of link 2 [m]
l3          = params.l3;            % Length of link 3 [m]
lcm1        = params.lcm1;          % CM position of link 1 [m]
lcm2        = params.lcm2;          % CM position of link 2 [m]
m1          = params.m1;            % Mass of link 1 [kg]
m2          = params.m2;            % Mass of link 2 [kg]
m3          = params.m3;            % Mass of link 3 [kg]
Icm1        = params.I_lcm1;        % Moment of inertia of link 1 [kg.m^2]
Icm2        = params.I_lcm2;        % Moment of inertia of link 2 [kg.m^2]
Icm3        = params.I_lcm3;        % Moment of inertia of link 2 [kg.m^2]
kr1         = params.kr1;           % Gear ratio of motor at joint 1 [-]
kr2         = params.kr2;           % Gear ratio of motor at joint 2 [-]
kr3         = params.kr3;           % Gear ratio of motor at joint 2 [-]
M1          = params.m_motor1;      % Mass of rotor at joint 1 [kg]
M2          = params.m_motor2;      % Mass of rotor at joint 2 [kg]
M3          = params.m_motor3;      % Mass of rotor at joint 2 [kg]
I_M1        = params.I_motor1;      % Moment of inertia of rotor at joing 1 [kg.m^2]
I_M2        = params.I_motor2;      % Moment of inertia of rotor at joing 2 [kg.m^2]
I_M3        = params.I_motor3;      % Moment of inertia of rotor at joing 2 [kg.m^2]
g           = params.g;             % Gravity [m/s^2]
Io1         = Icm1+m1*lcm1^2;     % Moment of inertia of link 1 around origin [kg.m^2]

%% Dynamics

% B(theta)*theta_ddot + C(theta,theta_dot)*theta_dot + Fv*theta_dot + Fs*sign(theta_dot) + g(theta) = u - J'(theta)*he

b11 = 32.5+m2*l1^2;
b12 = 10+m2*l2*lcm2*cos(theta2);
b13 = 0;
b21 = b12;
b22 = 10+m2*lcm2^2;
b23 = 0;
b31 = 0;
b32 = 0;
b33 = 10+10*cos(theta1+theta2)^2+cos(theta1)^2*(22.5+m2*l1^2);
  
c11 = 0;
c12 = -m2*l2*lcm2*sin(theta2)*theta2_dot;
c13 = (sin(theta1)*cos(theta1)*22.5+m2*l1^2+10*sin(theta1+theta2)*...
    cos(theta1+theta2))*theta3_dot;
c21 = 2*m2*l1*lcm2*sin(theta2)*theta2_dot;
c22 = 0;
c23 = 10*sin(theta1+theta2)*cos(theta1+theta2)*theta3_dot;
c31 = 0;
c32 = 0;
c33 = -2*cos(theta1)*sin(theta1)*theta1_dot*(22.5+m2*l1^2)-20*sin(theta1+theta2)...
    *cos(theta1+theta2)*(theta1_dot+theta2_dot);

g11 = m1*g*lcm1*cos(theta1)+m1*g*l1*cos(theta1)+m2*g*lcm2*cos(theta1+theta2);
g21 = m2*g*lcm2*cos(theta1+theta2);
g31 = 0;

B_theta = [b11 b12 b13; 
           b21 b22 b23;
           b31 b32 b33];

C_theta_theta_dot = [c11 c12 c13; 
                     c21 c22 c23;
                     c31 c32 c33];
            
G_theta = [g11;g21;g31];
  
%% Control Law        
u = zeros(3,1);

Theta_DOT = [theta1_dot; theta2_dot; theta3_dot];
Theta_DDOT = pinv(B_theta)*[ u - C_theta_theta_dot*Theta_DOT - G_theta ];

%% 

X_dot(1,1) = theta1_dot;
X_dot(2,1) = Theta_DDOT(1);
X_dot(3,1) = theta2_dot;
X_dot(4,1) = Theta_DDOT(2);
X_dot(5,1) = theta3_dot;
X_dot(6,1) = Theta_DDOT(3);

end