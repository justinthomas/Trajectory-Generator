function [] = PlotTraj(traj)
  % PlotTraj(traj)

warning off MATLAB:legend:IgnoringExtraEntries

if iscell(traj)
    keytimes = traj{1}.keytimes;
else
    keytimes = traj.keytimes;
end

tstep = (keytimes(end) - keytimes(1))/100;
tvec = keytimes(1):tstep:keytimes(end);

val = TrajEval(traj, tvec, 0:2);
ndim = size(val, 2);
nderiv = size(val, 3);

h = zeros(ndim,1);
for idx = 1:ndim
    
    h(idx) = figure;
    
    % Make the rows be timesteps and the columns to be derivatives
    plot(tvec, permute(val(:,idx,:), [1, 3, 2]), 'LineWidth', 2);
    
    title(['Dimension ', num2str(idx)])
    xlabel('time (s)')
    ylabel('value')
    legend('Value', 'd/dt', 'd^2/dt^2', 'd^3/dt^3', 'd^4/dt^4')
end

switch ndim
    case 2
        % Assume x and z
        plot(val(:,1,1), val(:,2,1));
        hold all;
        plot(val(:,1,1), val(:,1,2));
        plot(val(:,1,1), val(:,2,2));
        legend('Z', 'X Velocity', 'Z Velocity');
        xlabel('X Position');
        ylabel('Z Position, X Velocity, or Z Velocity');
end

drawnow;

end