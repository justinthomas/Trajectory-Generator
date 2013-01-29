function [] = PlotTraj(traj)

warning off MATLAB:legend:IgnoringExtraEntries

tstep = (traj.keytimes(end) - traj.keytimes(1))/1000;
tvec = traj.keytimes(1):tstep:traj.keytimes(end);

val = TrajEval(traj, tvec);
ndim = size(val, 2);
nderiv = size(val, 3);

h = zeros(ndim,1);
for idx = 1:ndim
    h(idx) = figure;
    
    % Make the rows be timesteps and the columns to be derivatives
    plot(tvec, permute(val(:,idx,:), [1, 3, 2]));
    
    title(['Dimension ', num2str(idx)])
    xlabel('time (s)')
    ylabel('value')
    legend('Value', 'd/dt', 'd^2/dt^2', 'd^3/dt^3', 'd^4/dt^4')

    pause;
    close
end

end