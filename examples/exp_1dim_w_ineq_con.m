clear
close all
clc

% Dimension
d = 1;

% The time vector
t = 0:10:10;

%% Waypoints

% Trajectory Start
waypoints(1) = ZeroWaypoint(t(1), d);
waypoints(1).pos(1) = 0;

% Start of bottom
waypoints(2) = ZeroWaypoint(t(2), d);
waypoints(2).pos(1) = 0;

%% Options and Bounds

% Set the options
options = {'ndim',d ,'order',12, 'minderiv',4};

% Specify some bounds
bounds(1) = SetBound([2.5, 7.5], 'pos', 'lb', .5);
bounds(2) = SetBound([], 'pos', 'ub', 1);

%% Solve

% Generate the trajectory
traj = trajgen(waypoints, options, bounds);

%% Plotting

PlotTraj(traj)
