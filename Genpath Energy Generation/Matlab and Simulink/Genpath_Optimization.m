clear all;
close all;
clc;
%% Set Parameter
r1 = 0.75*10^-2 ; % m
r2 = 3*10^-2 ;  % m
k = 350  ; % N/m
Jp = 10^(-5) ; % kg*m^2
m = 0.16 ; % kg
Rg = 50 ; % Ohm
L = 1*10^-3 ;% H
Kt = 0.048 ;% V*s/rad

%J = 1/8*p*L*D^4 so Jg = Jp*(r1^4/r2^4) , J = a* r^4 ; a = 1/(2^7)*p*L Assume L
% is equivalent
Jg = Jp*(r1^4/r2^4) ; % N/m^2
a = Jp/r2^4 ;

%Assume 
Rl = 500 ; % Ohm 
% M from paper
M = m+((Jp)/(r2^2))+((Jg)/(r1^2)) ; % kg
M = m+((a*r2^4)/(r2^2))+((a*r1^4)/(r1^2)) ; % kg
M = m+(a*r2^2)+(a*r1^2) ; % kg





%% Run Simulink
t = linspace(0,0.8,1000) ;
sim('Genpath_Simulink',t);
%% Calculate Power Output 
%Vg = Kt*wg; % Volt
Irms = rms(I); % Amp
Pavg = Irms^2*Rl; % Watt
fprintf('Power generate = %6.8f Watt\n',Pavg);

%% Loop for various k Assume minimum at 50 N/m
Pvar = 0; % For collecting Pavg
kvar = linspace(50,1000,200); 
for i = 1:200 
    k = kvar(i);
    sim('Genpath_Simulink',t);
    Pvar(i) = rms(I)^2*Rl;
end
figure() ;
plot(kvar,Pvar);
xlabel('k [N/m^2]');
ylabel('Pavg [Watt]');
title('Pavg at various k');
yindex = find(Pvar==max(Pvar)) ;
k = kvar(yindex);
%% Loop for various m
Pvar = 0; % For collecting Pavg
mvar = linspace(0.01,5,200); 
for i = 1:200 
    m = mvar(i);
    sim('Genpath_Simulink',t);
    Pvar(i) = rms(I)^2*Rl;
end
figure() ;
plot(mvar,Pvar);
xlabel('m [kg]');
ylabel('Pavg [Watt]');
title('Pavg at various m');
yindex = find(Pvar==max(Pvar)) ;
m = mvar(yindex);
%% Loop for various L
Pvar = 0; % For collecting Pavg
Lvar = linspace(0.001,0.01,100); 
for i = 1:100 
    L = Lvar(i);
    sim('Genpath_Simulink',t);
    Pvar(i) = rms(I)^2*Rl;
end
figure() ;
plot(Lvar,Pvar);
xlabel('L [H]');
ylabel('Pavg [Watt]');
title('Pavg at various L');
yindex = find(Pvar==max(Pvar)) ;
L = Lvar(yindex);


%% Loop for various r1
Pvar = 0; % For collecting Pavg
r1var = linspace(0.0001,0.0499,50); 
for i = 1:50 
    r1 = r1var(i);
    sim('Genpath_Simulink',t);
    Pvar(i) = rms(I)^2*Rl;
end
figure() ;
plot(r1var,Pvar);
xlabel('r1 [m]');
ylabel('Pavg [Watt]');
title('Pavg at various r1');
yindex = find(Pvar==max(Pvar)) ;
r1 = r1var(yindex);
%% Loop for various r2
Pvar = 0; % For collecting Pavg
r2var = linspace(0.0001,0.0499,50); 
for i = 1:50 
    r2 = r2var(i);
    sim('Genpath_Simulink',t);
    Pvar(i) = rms(I)^2*Rl;
end
figure() ;
plot(r2var,Pvar);
xlabel('r2 [m]');
ylabel('Pavg [Watt]');
title('Pavg at various r2');
yindex = find(Pvar==max(Pvar)) ;
r2 = r2var(yindex);
%% Loop for various RL and optimize
Pvar = 0; % For collecting Pavg
Rlvar = linspace(10,1500,100); 
for i = 1:100 
    Rl = Rlvar(i);
    sim('Genpath_Simulink',t);
    Pvar(i) = rms(I)^2*Rl;
end
figure() ;
plot(Rlvar,Pvar);
xlabel('RL [Ohm]');
ylabel('Pavg [Watt]');
title('Pavg at various RL');
yindex = find(Pvar==max(Pvar)) ;
Rl = Rlvar(yindex);
fprintf('Maximum Power generate = %6.8f Watt\n',max(Pvar));
fprintf('It is optimize at\n');
fprintf('Load Resistance = %6.8f Ohm\n',Rl);
fprintf('Pinion Radius = %6.8f m\n',r2);
fprintf('Pinion of Generator Radius = %6.8f m\n',r1);
fprintf('Inductance = %6.8f H\n',L);
fprintf('Mass = %6.8f kg\n',m);
fprintf('Stiffness of spring = %6.8f N/m\n',k);

