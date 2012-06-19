% this function will be an example of the inputs to the trajectory
% generation mex file

clear all
close all
clc

addpath('../src/');
addpath('../matlab/');


% create a sequence of waypoints
waypoints(1) = ZeroWaypoint(0);
waypoints(1).pos = [0;0;2];

waypoints(2) = SetWaypoint(1,'pos',[0;nan;3]);

waypoints(3) = ZeroWaypoint(3);
waypoints(3).pos = [0;1;2];


% create a set of bounds
bounds(1) = SetBound([],'pos','lb',[-2,-2,0]);
bounds(2) = SetBound([0,1],'pos','ub',[2,2,4]);


% call the trajectory function
trajectory = trajgen(waypoints,bounds,options);


