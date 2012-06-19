function output = ZeroWaypoint(time)

if nargin<1
    time = nan;
end

output = struct('time',{time},'pos',{[0;0;0]},'vel',{[0;0;0]},'acc',{[0;0;0]},'jerk',{[0;0;0]},'psi',{[0;0;0]});