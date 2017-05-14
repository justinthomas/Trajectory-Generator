clear
close all
clc

% Dimension
d = 4;

% The time vector
t = [0, 1];

%% Waypoints

% Trajectory Start
waypoints(1) = ZeroWaypoint(t(1), d);
waypoints(1).pos = [1; 1; 2; 0];

% Trajectory End
waypoints(2) = ZeroWaypoint(t(2), d);
waypoints(2).pos = waypoints(1).pos + [1; 1; 0; 0];

%% Options and Bounds

% Set the options
options = {'order',9, 'ndim',d, 'minderiv',[4,4,4,2], 'contderiv',[4,4,4,4], 'convergetol', 1e-9};

% Specify some bounds
bounds = [];

%% Solve

% Generate the trajectory
traj = trajgen(waypoints, options, bounds);

%% Plotting

PlotTraj(traj)