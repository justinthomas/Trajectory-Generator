clear all
close all
clc

addpath('../src/');
addpath('../matlab/');


% this function will be an example of the inputs to the trajectory
% generation mex file



waypoints(1) = EmptyWaypoint(0);
waypoints(2) = NanWaypoint(1);
waypoints(1) = EmptyWaypoint(0);




% trajectory = trajgen(waypoints,bounds,options);


