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

d = 4;
n = 9;

% create a sequence of waypoints
waypoints(1) = ZeroWaypoint(0,d);
waypoints(1).pos = [1;0;2; 0];
% waypoints(1).pos = 1;

waypoints(2) = SetWaypoint(1,'pos',[0;nan;3;0]);
% waypoints(2) = SetWaypoint(1,'pos',2);
% waypoints(2) = SetWaypoint(1,'vel',2);

waypoints(3) = ZeroWaypoint(3,d);
waypoints(3).pos = [0;1;2;0];
waypoints(3).vel = [0; 0; 0; 0];
waypoints(3).acc = [0; 0; 0; NaN];
waypoints(3).jerk = [0; 0; 0; NaN];


% create a set of bounds
bounds(1) = SetBound([],'pos','lb',[-.3,-2,0,nan]);
bounds(2) = SetBound([0,1],'pos','ub',[2,2,4,nan]);
bounds(3) = SetBound([1.5, 3], 'vel', 'ub', [nan, .3, nan, nan]);


% bounds(1) = SetBound([1 2],'pos','ub',2);
% bounds(2) = SetBound([.5 1],'pos','lb',1.2);

% bounds = [];

options = {'ndim',d,'polyorder', n,'minderiv', [4 4 4 2]};

% call the trajectory function
tic
[trajectory problem] = trajgen(waypoints,options, bounds, options);
toc


colors = distinguishable_colors(length(waypoints)-1);
figure
set(gcf, 'Position', [1, 57, 1280, 945]);
for didx = 1:d
    subplot(2,2,didx)
    hold on
    title(sprintf('Dimension: %d', didx));
    
    for seg = 1:(length(waypoints)-1)
        t = waypoints(seg).time:.01:waypoints(seg+1).time;
        rows = ((didx-1) + d*(seg - 1))*(n+1) + 1;
        plot(t, polyval(trajectory(rows:(rows+n)),t), 'Color', colors(seg,:), 'LineWidth', 3)
    end
    
end