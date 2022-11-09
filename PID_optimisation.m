%% Init

if(isfile("appconfig.mat"))
    load("appconfig.mat");                                                  %Load variables if not
end

scootermodelsim;
num_run = 0;                                                                %Simulation number
%Change these if using PID optimization, note that combinations easily can
%increase, and thus computation time. ---
%PID sample distance
sample = 0.1;
%Outer PID sweep settings
outer_p = 3.8:sample:4.1;
outer_i = 0.2:sample:0.2;
outer_d = 0.2:sample:0.2;
%Inner PID sweep settings
inner_p = 4.5:sample:5.1;
inner_i = 0:sample:0;
inner_d = 1:sample:1;
%---
half_error_band = 0.003;                                                    %Stabilisation definition (settled when the angle is within error band)
startt = 2*(1/Ts);                                                          %Start time for settling time search (do not put lower than 1/Ts = 1s for this version)
endt = simulationtime*(1/Ts);                                               %Times 1/Ts = 100 is for simOut (1500 samples = 15s)
timeresolution = 0.1*(1/Ts);                                                %Settling graph sampling time for settling assessment
analysetime = 3*(1/Ts);                                                     %Time window for settling assessment, if stable within [t,t+analysetime] then t is considered settling time

settling_graph = zeros(((endt-startt)/timeresolution)+1,1);                 %Init

%% Parallel simulation

save_system('scootermodelsim');                                             %Simulation needs to be saved before parallel

%Run the simulation parallel
parpool(6);                                                                 %Open pool of specific size, MATLAB allows 6 max

%One PID parallel model
if PIDswitch == 0
    %PID combinations---
    for iP = 1:1:length(outer_p)
        for iI = 1:1:length(outer_i)
            for iD = 1:1:length(outer_d)
            %---
                num_run = num_run + 1;                                      %One more simulation run
                %PID values sent to Simulink scooter simulation
                in(num_run) = Simulink.SimulationInput('scootermodelsim');
                in(num_run) = setVariable(in(num_run),'outer_p',outer_p(iP));
                in(num_run) = setVariable(in(num_run),'outer_i',outer_i(iI));
                in(num_run) = setVariable(in(num_run),'outer_d',outer_d(iD));
            end
        end
    end
%Two PIDs parallel model
else
    %PID combinations---
    for iP = 1:1:length(outer_p)
        for iI = 1:1:length(outer_i)
            for iD = 1:1:length(outer_d)
                for iP2 = 1:1:length(inner_p)
                    for iI2 = 1:1:length(inner_i)
                        for iD2 = 1:1:length(inner_d)
                        %---
                            num_run = num_run + 1;                          %One more simulation run
                            %PID values sent to Simulink scooter simulation
                            in(num_run) = Simulink.SimulationInput('scootermodelsim');
                            in(num_run) = setVariable(in(num_run),'outer_p',outer_p(iP));
                            in(num_run) = setVariable(in(num_run),'outer_i',outer_i(iI));
                            in(num_run) = setVariable(in(num_run),'outer_d',outer_d(iD));
                            in(num_run) = setVariable(in(num_run),'inner_p',inner_p(iP2));
                            in(num_run) = setVariable(in(num_run),'inner_i',inner_i(iI2));
                            in(num_run) = setVariable(in(num_run),'inner_d',inner_d(iD2));
                        end
                    end
                end
            end
        end
    end
end

set_param('scootermodelsim','SimMechanicsOpenEditorOnUpdate','off');        %If turned on, scooter 3D model will run for all PID combinations (not recommended)
save_system('scootermodelsim');                                             %Simulation needs to be saved before parallel
%Output of all combinations chosen
simOut = parsim(in, 'ShowSimulationManager', 'off', 'TransferBaseWorkspaceVariables', 'on')
delete(gcp('nocreate'));                                                    %Quit parallel functionality

%% PID optimization function

%One PID settling time
if PIDswitch == 0
    num_run = 0;
    %PID combinations---
    for iP = 1:1:length(outer_p)
        for iI = 1:1:length(outer_i)
            for iD = 1:1:length(outer_d)
            %---
                num_run = num_run + 1;                                      %One more simulation run
                count = 0;
                for t = startt:timeresolution:endt
                    %Pick discretised values to describe settling graph
                    %(steer angle graph with lower sampling freq.)
                    count = count + 1;
                    settling_graph(count,1) = simOut(1,num_run).roll_angle_CAD(t,1);
                end
    
                for t = startt/timeresolution:1:(endt-analysetime)/timeresolution
                    %Check when (and if) the settling graph settles
                    settled = 1;
                    for move = 1:1:analysetime/timeresolution
                        %If not within error band at some occasion between
                        %t and analysetime
                        if abs(settling_graph(t-(startt/timeresolution)+move,1)) > half_error_band
                            settled = 0;                                    %Not settled yet
                        end
                    end
                    if settled == 1
                        %The graph settled at time t
                        settling_time(num_run,1) = t/timeresolution;
                        break
                    end
                end
    
                %The graph did not settle
                if settled == 0
                    settling_time(num_run,1) = NaN;                         %"Max/worst settling time"
                end

                %Print out PID combination
                settling_time(num_run,2) = outer_p(iP);
                settling_time(num_run,3) = outer_i(iI);
                settling_time(num_run,4) = outer_d(iD);
            end
        end
    end
    %Get best performing PID combination (best settling time)
    [best_PID(1,1),best_PID_index] = min(settling_time(:,1));
    best_PID(1,2:4) = settling_time(best_PID_index,2:4);
%Two PIDs settling time
else
    num_run = 0;
    %PID combinations---
    for iP = 1:1:length(outer_p)
        for iI = 1:1:length(outer_i)
            for iD = 1:1:length(outer_d)
                for iP2 = 1:1:length(inner_p)
                    for iI2 = 1:1:length(inner_i)
                        for iD2 = 1:1:length(inner_d)
                        %---
                            num_run = num_run + 1;                          %One more simulation run
                            count = 0;
                            for t = startt:timeresolution:endt
                                %Pick discretised values in some time to describe settling
                                %graph (steer angle graph with lower sampling freq.)
                                count = count + 1;
                                settling_graph(count,1) = simOut(1,num_run).roll_angle_CAD(t,1);
                            end
                
                            for t = startt/timeresolution:1:(endt-analysetime)/timeresolution
                                %Check when (and if) the settling graph settles
                                settled = 1;
                                for move = 1:1:analysetime/timeresolution
                                    %If not within error band at some occasion between
                                    %t and analysetime
                                    if abs(settling_graph(t-(startt/timeresolution)+move,1)) > half_error_band
                                        settled = 0;                        %Not settled yet
                                    end
                                end
                                if settled == 1
                                    %The graph settled at time t
                                    settling_time(num_run,1) = t/timeresolution;
                                    break
                                end
                            end
                
                            if settled == 0 %The graph did not settle
                                settling_time(num_run,1) = NaN;             %"Max/worst settling time"
                            end

                            %Print out PID combination
                            settling_time(num_run,2) = outer_p(iP);
                            settling_time(num_run,3) = outer_i(iI);
                            settling_time(num_run,4) = outer_d(iD);
                            settling_time(num_run,5) = inner_p(iP2);
                            settling_time(num_run,6) = inner_i(iI2);
                            settling_time(num_run,7) = inner_d(iD2);
                        end
                    end
                end
            end
        end
    end
    %Get best performing PID combination (best settling time)
    [best_PID(1,1),best_PID_index] = min(settling_time(:,1));
    best_PID(1,2:7) = settling_time(best_PID_index,2:7);
end

%Best graph used for plot in GUI. If no plot stabilises during specified
%time (simulationtime-(analyzetime/(1/Ts))), no graph will be plotted
bestgraph=simOut(1,best_PID_index);
save("best_PID.mat","best_PID","bestgraph");

%Save PID combinations characteristics
save("simOut.mat","simOut");
save("settling_time.mat","settling_time");
