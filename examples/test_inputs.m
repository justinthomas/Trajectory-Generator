% this function will be an example of the inputs to the trajectory
% generation mex file

clear
close all
clc

% mex ../src/trajgen.c

% addpath('../src/');
% addpath('../matlab/');
% addpath('./matlab/');
% addpath('./src/');

d = 1;
n = 12;

if isequal(d, 4)
    
    t = [0 2 5 8];
    
    % create a sequence of waypoints
    waypoints(1) = ZeroWaypoint(t(1),d);
    waypoints(1).pos = [nan; 0; 2; (pi/2)*8/10];
    
    waypoints(2) = SetWaypoint(t(2),'pos',[0 0 0 0]);
    waypoints(2).vel = [dbeta_max/2*L; nan; 0; -dbeta_max];
    
    waypoints(3) = SetWaypoint(t(3),'pos',[1; nan; 2; nan]);
    
    waypoints(4) = ZeroWaypoint(t(end),d);
    waypoints(4).pos = [nan; nan; 2; 0];
    
    
    % create a set of bounds
    bounds(1) = SetBound([],'pos','ub',[2; 2; 2.5; pi/2]);
    bounds(2) = SetBound([],'pos','lb',[-2; -2; 0; -pi/5]);
    bounds(3) = SetBound([],'vel','ub',[nan; nan; nan; dbeta_max]);
    bounds(4) = SetBound([],'vel','lb',[nan; nan; nan; -dbeta_max]);

    minderiv = [4 4 4 2];
    
elseif isequal(d,1)
    
    t = [0 5 10]/10;
    
    waypoints(1) = NanWaypoint(t(1),d);
    waypoints(1).pos = 1;
    waypoints(1).vel = 0;
    
    waypoints(2) = NanWaypoint(t(2),d);
    waypoints(2).pos = -1;
    
    waypoints(3) = NanWaypoint(t(3),d);
    waypoints(3).pos = 1;
    waypoints(3).vel = 0;
    
    minderiv = 2;
    
end

bounds = [];

options = {'ndim',d,'order', n,'minderiv', minderiv, 'constraints_per_seg', 20*(n+1)};

% call the trajectory function
tic
[traj durations problem] = trajgen(waypoints, options, bounds);
toc

PlotTraj(traj)
% 
% N = length(waypoints)-1;
% colors = distinguishable_colors(2*N);
% 
% set(gcf, 'Position', [1, 57, 1280, 945]);
% for didx = 1:d
%     if d>2
%         subplot(2,2,didx)
%     end
%     
%     hold on
%     title(sprintf('Dimension: %d', didx));
%     
%     for seg = 1:N
%         t = 0:.01:(waypoints(seg+1).time - waypoints(seg).time);
%         plot(t+waypoints(seg).time, polyval(traj.poly(:,didx,seg),t), 'Color', colors(seg,:), 'LineWidth', 4);
% %         plot(t+waypoints(seg).time,...
% %             polyval(polyder(traj.poly(:,didx,seg)),t), 'Color',colors(seg,:), 'LineWidth',2);
% %         plot(t+waypoints(seg).time,...
% %             polyval(polyder(polyder(polyder(traj(:,didx,seg)))),t), 'Color',colors(seg,:), 'LineWidth',1);
%     end
%     
% end