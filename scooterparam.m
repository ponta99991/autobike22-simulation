
clear all
clc

Simulink.sdi.clear %Clear simulink data inspector


%scooter translation and rotation from CAD
scootermodel_DataFile;


%% scooter  parameters(From chalmers bike)-----------------
    r_wheel = 0.11;        % radius of the wheel
    h = 0.27;   % height of center of mass [m]
    b = 0.835;              % length between wheel centers [m]
    a = 0.515;             % distance from rear wheel to frame's center of mass [m]

%% additional parameters

PIDswitch=0; %0 for 1 and 1 for 2 PIDs
Pidopton = 0; %0 = normal use, 1 = PID settling time optimization results

zeta = 0.6;
d = 0.015; %Time delay
v = 8; %Velocity (m/s) (model assumes constant velocity)

%Different sample rates
Ts=0.01;
Tsm=Ts/6;

simulationtime=10;
% Control for transfer function
%Starts with o=outer loop, i=inner loop
% PID settings if running once (not using PID optimization calculation)
outer_p = 4;
outer_i = 0.2;
outer_d = 0.2;

inner_p = 5;
inner_i = 0;
inner_d = 0.1;
N = 10;


%% Test parameters
%disturbance
pushamp=5;
phase=2;

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
initvelocity=v/(r_wheel*3.6); %rad/s (w = v/r)

%The scooter is not aligned at plane height such 20 cm dropheight is used
%to have it over, future work to allign them better
dropheight=20; %cm

%smlink_linksw %README before calling this(only if a model needs to be updated)
%smimport('scootermodel'); %README before calling this(only if a model needs to be updated)

tic;

if Pidopton == 0 %Run once
    %run the simulation
   % set_param('scootermodelsim','SimMechanicsOpenEditorOnUpdate','off');
   % simOut = sim("scootermodelsim");
    
else
    PID_optimisation
    
%     %PID optimization function
%     for P = Kp
%         for I = Ki
%             for D = Kd
%                 count = 0;
%                 for t = startt:timeresolution:endt
%                     %Pick discretised values in some time to describe settling
%                     %graph (steer angle graph with lower sampling freq.)
%                     count = count + 1;
%                     settling_graph(count,1) = out.steer_angle_CAD(t,1);
%                 end
%     
%                 for t = startt/10:timeresolution/10:(endt-analyzetime)/10
%                     %Check when (and if) the settling graph settles
%                     settled = 1;
%                     for move = 1:1:analyzetime/timeresolution
%                         if abs(settling_graph(t-(startt/10)+move,1)) > half_error_band
%                             settled = 0; %Not settled yet
%                         end
%                     end
%                     if settled == 1
%                         settling_time(configuration,1) = t/timeresolution; %The graph settled at time t
%                         settling_time(configuration,2) = Kp;
%                         settling_time(configuration,3) = Ki;
%                         settling_time(configuration,4) = Kd;
%                         break
%                     end
%                 end
%     
%                 if settled == 0 %The graph did not settle
%                     settling_time(configuration,1) = t/timeresolution; %"Max/worst settling time" (it is actually higher than t)
%                     settling_time(configuration,2) = Kp;
%                     settling_time(configuration,3) = Ki;
%                     settling_time(configuration,4) = Kd;
%                 end
%     
%                 configuration = configuration + 1;
%     
%             end
%         end
%     end
%     
%     [best_PID(1,1),best_PID_index] = min(settling_time(:,1));
%     best_PID(1,2:4) = settling_time(best_PID_index,2:4);
%     
%     % Settling time and its PID values as excel sheet
%     xlswrite('settling_time.xlsx',settling_time);

end

computing_time = toc;

