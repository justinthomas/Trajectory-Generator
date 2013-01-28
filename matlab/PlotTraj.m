function [] = PlotTraj(traj)

tstep = (traj.keytimes(end) - traj.keytimes(1))/1000;
tvec = traj.keytimes(1):tstep:traj.keytimes(end);

val = TrajEval(traj, tvec);
ndim = size(val, 2);

h = zeros(ndim,1);
for idx = 1:ndim
    h(idx) = figure;
    
    plot(...
        tvec, val(:,idx,1),... % Position
        tvec, val(:,idx,4),... % Jerk
        tvec, val(:,idx,5));   % Snap
    
    title(['Dimension ', num2str(idx)])
    xlabel('time (s)')
    ylabel('value')
    legend('Position', 'Jerk', 'Snap')
    pause;
    close
end

end