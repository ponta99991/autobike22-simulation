clear all
clc

Simulink.sdi.clear %Clear simulink data inspector

%% scooter  parameters(From chalmers bike)-----------------
    %inertia_front = 0.245;  %[kg.m^2] NOT SET, Needs to be calculated?
    r_wheel = 0.11;        % radius of the wheel
    h = 0.27;   % height of center of mass [m]
    b = 0.835;              % length between wheel centers [m]
    %c = 0.02;               % length between front wheel contact point and the extention of the fork axis [m]
    %lambda = deg2rad(70);   % angle of the fork axis [deg]
    a = 0.515;             % distance from rear wheel to frame's center of mass [m]
    % IMU_height = 0.45;      % % NOT SET, this could vary, pick a suitable value.
    %m = 45;                   % NOT SET, Not measured, consult online sources (or guess)
    %uneven_mass = 0;        % 1 = use uneven mass distribution in bike model ; 0 = do not use

% Notes:
% 0.14m from ground to tread plate
% 0.95m from tread plate to handle bar

%% P(s) additional parameters

zeta = 0.6;
d = 0.015; %Time delay
v = 8/3.6; %Velocity (m/s) (model assumes constant velocity)

%Different sample rates
Ts=0.01;
Tsm=Ts/6;

% Control for transfer function
Kp = 2.7;
Ki = 0.2;
Kd = 0.2;
N = 100;


%% Test parameters

%standard friction values on back and front wheel in static and dynamic
%states, find these in contactpoints block.
Friction_stat_front=0.8;
Friction_dyn_front=0.7;
Friction_stat_back=0.8;
Friction_dyn_back=0.7;

%on The revolute joints damping coefficient and spring stiffness is available both for back and front wheel. 
damping_front=0.1; %N*m/(deg/s) 
spring_front=0;    %N*m/deg

damping_back=0.1; %N*m/(deg/s)
spring_back=0;    %N*m/deg



%% Plane parameters------- 

xPla = 80;     % x plane length [m]
yPla = 40;      % y plane length [m]

zPla = 0.01;   % z plane depth [m]
surfacex=0:1:xPla;
surfacey=0:1:yPla;

%% Dont change below paramters without motivation-------------
%rotates the frames such Z is up also sets gravity in z
rotationaxis=[1 0 0];
rotationangle=pi/2;
gravity = 9.82;        % gravity [m/s^2]
%%Don't change above
initvelocity=v/r_wheel; %rad/s (w = v/r)

%The scooter is not aligned at plane height such 20 cm dropheight is used
%to have it over, future work to allign them better
dropheight=20; %cm

%% Transfer functions (P(s))

%steering system step response matching
wn=33.9; %measured value (rad/s)
steer_sys = tf(wn^2,[1,2*zeta*wn,wn^2],'InputDelay',d); %H(s)
sys_discretePID=c2d(steer_sys,Tsm,'matched'); %discrete time

%Transfer function for point mass model from steering pos to lean angle
sys = tf((a*v/(h*b))*[1,v/a],[1,0,-gravity/h]); %G(s)
%runs in continous time

%%

%smlink_linksw %README before calling this(only if a model needs to be updated)
%smimport('scootermodel'); %README before calling this(only if a model needs to be updated)

%scooter translation and rotation from CAD
scootermodel_DataFile 
%run the simulation
simOut = sim("scootermodelsim");

%% Additional plot for comparison between CAD/IMU and TF

% figure
% axes('FontSize', 30, 'NextPlot', 'add');
% plot(0:0.01:30, simOut.steer_angle_CAD, 0:0.01:30, simOut.steer_angle_TF, 'LineWidth', 2)
% title('Steer angles from CAD/IMU and Transfer Function model','FontSize',30)
% xlabel('t (s)','FontSize',30)
% ylabel('Steer angle (rad)','FontSize',30)
% legend({'CAD/IMU','TF'},'FontSize',35)

%  figure
%  axes('FontSize', 30, 'NextPlot', 'add');
%  plot(0:0.01:30, simOut.steer_angle_CAD, 'LineWidth', 2)
%  title('Steer angle from CAD/IMU','FontSize',30)
%  xlabel('t (s)','FontSize',30)
%  ylabel('Steer angle (rad)','FontSize',30)
%  legend('CAD/IMU','FontSize',35)

% figure
% axes('FontSize', 30, 'NextPlot', 'add');
% plot(0:0.01:30, simOut.roll_angle_CAD, 'LineWidth', 2)
% title('Roll angle from CAD/IMU model','FontSize',30)
% xlabel('t (s)','FontSize',30)
% ylabel('Roll angle (rad)','FontSize',30)
% legend({'CAD/IMU','TF'},'FontSize',35)

