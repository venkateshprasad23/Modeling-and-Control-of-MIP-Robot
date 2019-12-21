% physical constants
r = 0.034; % wheel radius m
l = 0.036; % center of mass to axle m
m_w = 0.027; % wheel mass kg
m_b = 0.263; % body mass kg
G_r = 35.57; % gearbox ratio
I_m = 3.6e-8; % motor armature moment of inertia kgm^2
I_b = 4e-4; % body moment of inertia
V_max = 7.4; % max battery voltage V
s = 0.003; % motor stall torque Nm
omega_f = 1760; % motor free run speed rad/sec
g = 9.8; % acceleration due to gravity m/s^2

% derived quantities
I_w = m_w*r^2/2+G_r^2*I_m; % momnet f interia of singel wheel plus gearbox
k = s/omega_f; % motor constant

% intermediate cosntants
a = 2*I_w+(m_b+2*m_w)*r^2;
b = m_b*r*l;
c = I_b+m_b*l^2;
d = m_b*g*l;
e = 2*G_r*s/V_max;
j = 2*G_r^2*k;
XX = 1/(a*c-b*b);
i =  sqrt(-1);

% Linearized matrices
A = [-(a+b)*j*XX (a+b)*j*XX a*d*XX;(b+c)*j*XX -(b+c)*j*XX -b*d*XX;1 0 0]
B = [-(a+b)*e*XX; (b+c)*e*XX; 0]
C = [1 0 0; 0 1 0]

% Zhu Zhuo's second-order discrete controller
ALCBKDmIp = [0.9129 0.0378;-0.0655 0.9933]
LLDmIp = [0.002835 -0.0005967;0.001463 -0.001286]
KLDmIp = [-380.821 322.4448]
DLDmIp = [-1.24321 0]

% Linear State Estimate Feedback

% design parameters - Profs
%ceig = [-2 -2+2*i -2-2*i];    % A-BK eigenvalues
%oeig = [-6 -6.6 -5.8];      % A-LC eigenvalues

% design parameters - Mine(Good)
%ceig = [-4.2 -4.4 -4.6];    % A-BK eigenvalues
%oeig = [-6.4-3.2*i -6.4+3.2*i -6.4];      % A-LC eigenvalues

% design parameters - Mine(On Test)
%ceig = [-7.5+2.2*i -7.5-2.2*i -7.5];    % A-BK eigenvalues
%oeig = [-10.5+5.2*i -10.5-5.2*i -10.5];      % A-LC eigenvalues

% design parameters - Mine(Best)
%ceig = [-8+2.2*i -8-2.2*i -8.2];    % A-BK eigenvalues
%oeig = [-12+5.2*i -12-5.2*i -12];      % A-LC eigenvalues


% ceig = [-30+2.2*i -30-2.2*i -6.4];    % A-BK eigenvalues
% oeig = [-31+5.2*i -31-5.2*i -12];      % A-LC eigenvalues

ceig = [-98+8*i -98-8*i -0.001];    % A-BK eigenvalues(high roa)
oeig = [-31+5.2*i -31-5.2*i -12];      % A-LC eigenvalues


%ceig = [-2 -2+2*i -2-2*i];    % A-BK eigenvalues
%oeig = [-6 -6.6 -5.8];      % A-LC eigenvalues


% test parameter initial angle
thetaic = 0.91;%-0.914;%0.2;

% gains LSVF and observer
K=place(A,B,ceig);
L=place(A',C',oeig)';
% gains LSVF and observer
% K=lqr(A,B,[10,0,0;0,1,0;0,0,10000],0.01);
% L=lqe(A,0.001*ones(3,1),C,4,[0.01 0; 0 0.1]);
% continuous-time controller
cntrlr = ss(A-B*K-L*C,L,-K,[0 0]);
% discrete-time controller
dcntrlr  = c2d(cntrlr,0.01,'tustin');
% extract matrices for Simulink
[Amine,Lmine,Kmine,Dmine]=ssdata(dcntrlr);
% T=0:0.01:10;
% U=0.2*ones(size(T));
% [Y,X]=dlsim(Amine,Lmine,-Kmine,Dmine,U);
% stairs(T,Y)
% legend('Cart (x)','Pendulum (phi)')


