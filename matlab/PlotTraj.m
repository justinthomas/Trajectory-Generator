function [] = PlotTraj(traj)

tstep = (traj.keytimes(end) - traj.keytimes(1))/100;
tvec = traj.keytimes(1):tstep:traj.keytimes(end);

val = TrajEval(traj, tvec);
ndim = size(val, 2);

h = zeros(ndim,1);
for idx = 1:ndim
    h(idx) = figure;
    plot(tvec, val(:,idx,1));
    title(['Dimension ', num2str(idx)])
    xlabel('time (s)')
    ylabel('position (m or rad)')
    pause;
    close
end

end