% this function will be an example of the inputs to the trajectory
% generation mex file

clear all
close all
clc

% mex ../src/trajgen.c

% addpath('../src/');
% addpath('../matlab/');
% addpath('./matlab/');
% addpath('./src/');

ndim = 4;

% create a sequence of waypoints
waypoints(1) = ZeroWaypoint(0,ndim);
waypoints(1).pos = [0;0;2; 0];

waypoints(2) = SetWaypoint(1,'pos',[0;nan;3;0]);

waypoints(3) = ZeroWaypoint(3,ndim);
waypoints(3).pos = [0;1;2;0];
waypoints(3).vel = [0; 0; 0; 0];
waypoints(3).acc = [0; 0; 0; NaN];
waypoints(3).jerk = [0; 0; 0; NaN];


% create a set of bounds
bounds(1) = SetBound([],'pos','lb',[-2,-2,0,nan]);
bounds(2) = SetBound([0,1],'pos','ub',[2,2,4,nan]);

options = {'ndim',ndim,'polyorder', 10,'minderiv', [4 4 4 2]};

% call the trajectory function
tic
trajectory = trajgen(waypoints,options, bounds, options);
toc

