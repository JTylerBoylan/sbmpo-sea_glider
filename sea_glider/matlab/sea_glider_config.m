%% Sea glider Benchmark

clear
close all

%% Parameters

runs = 1;

params = struct;
params.max_iterations = 750000;
params.max_generations = 100;
params.horizon_time = 1.0;
params.num_states = 10;
params.num_controls = 1;
% states = [x, y, theta, dx, dy, dtheta, SG, Energy, PCMT, t]
params.grid_resolution = [0.01; 0.01; -1; 0.05; 0.05; -1; -1; -1; -1; -1];
params.start_state = [0; -0.01; 0; 0; 0; 0; 1; 0; 0; 0];
params.goal_state = [0; 0; 0; 0; 0; 0; 0; 0; 0; 45];
params.branchout_factor = 3;
% controls
params.branchouts = linspace(-0.015,0.015,params.branchout_factor);
% params.branchouts = linspace(-0.005,0.005,params.branchout_factor);


%% Write config file

sbmpo_config("../csv/config.csv", params, runs);
