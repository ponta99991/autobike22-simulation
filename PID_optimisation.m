    load("appconfig.mat");
    scootermodelsim;
    endt = simulationtime*100; %times 100 is for simOut (1500)
    num_run = 0; %Simulation number
    outer_p = 2.5:0.1:2.5;
    outer_i = 0.1:0.1:0.1;
    outer_d = 0.1:0.1:0.3;
    half_error_band = 0.01;
    startt = 1*100;
    timeresolution = 0.1*100;
    analyzetime = 3*100;
    
    settling_graph = zeros(((endt-startt)/timeresolution)+1,1);

    %run the simulation
    parpool(6); %Open pool of specific size
    for iP = 1:1:length(outer_p)
        for iI = 1:1:length(outer_i)
            for iD = 1:1:length(outer_d)
                num_run = num_run + 1; %One more simulation run
                in(num_run) = Simulink.SimulationInput('scootermodelsim');
                in(num_run) = setVariable(in(num_run),'outer_p',outer_p(iP));
                in(num_run) = setVariable(in(num_run),'outer_i',outer_i(iI));
                in(num_run) = setVariable(in(num_run),'outer_d',outer_d(iD));
            end
        end
    end
    simOut = parsim(in, 'ShowSimulationManager', 'on', 'TransferBaseWorkspaceVariables', 'on')

    delete(gcp('nocreate'));

    %PID optimization function
    num_run = 0;
    for iP = 1:1:length(outer_p)
        for iI = 1:1:length(outer_i)
            for iD = 1:1:length(outer_d)
                num_run = num_run + 1; %One more simulation run
                count = 0;
                for t = startt:timeresolution:endt
                    %Pick discretised values in some time to describe settling
                    %graph (steer angle graph with lower sampling freq.)
                    count = count + 1;
                    settling_graph(count,1) = simOut(1,num_run).steer_angle_CAD(t,1);
                end

                for t = startt/10:timeresolution/10:(endt-analyzetime)/10
                    %Check when (and if) the settling graph settles
                    settled = 1;
                    for move = 1:1:analyzetime/timeresolution
                        if abs(settling_graph(t-(startt/10)+move,1)) > half_error_band
                            settled = 0; %Not settled yet
                        end
                    end
                    if settled == 1
                        settling_time(num_run,1) = t/timeresolution; %The graph settled at time t
                        settling_time(num_run,2) = outer_p(iP);
                        settling_time(num_run,3) = outer_i(iI);
                        settling_time(num_run,4) = outer_d(iD);
                        break
                    end
                end

                if settled == 0 %The graph did not settle
                    settling_time(num_run,1) = t/timeresolution; %"Max/worst settling time" (it is actually higher than t)
                    settling_time(num_run,2) = outer_p(iP);
                    settling_time(num_run,3) = outer_i(iI);
                    settling_time(num_run,4) = outer_d(iD);
                end
    
%                 configuration = configuration + 1;

            end
        end
    end

    [best_PID(1,1),best_PID_index] = min(settling_time(:,1));
    best_PID(1,2:4) = settling_time(best_PID_index,2:4);
    save("best_PID.mat","best_PID");