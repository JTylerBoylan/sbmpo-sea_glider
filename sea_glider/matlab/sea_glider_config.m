%% Sea glider Benchmark

clear
close all

%% Parameters

runs = 1;

params = struct;
params.max_iterations = 50000;
params.max_generations = 100;
params.horizon_time = 1.0;
params.num_states = 10;
params.num_controls = 1;
params.grid_resolution = [0.1; 0.1; pi/16; 0.01; 0.01; pi/32; 0.1; 0; 0; 0];
params.start_state = [0; -0.5; 0; 0; 0; 0; 1; 100; 0; 0];
params.goal_state = [0; 0; 0; 0; 0; 0; 0; 0; 0; 100];
params.branchout_factor = 3;
params.branchouts = linspace(-0.25,0.25,params.branchout_factor);


%% Write config file

sbmpo_config("../csv/config.csv", params, runs);
