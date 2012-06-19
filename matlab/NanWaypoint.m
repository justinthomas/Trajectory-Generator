function output = NanWaypoint(time)

if nargin<1
    time = nan;
end

output = struct('time',{time},'pos',{[nan;nan;nan]},'vel',{[nan;nan;nan]},'acc',{[nan;nan;nan]},'jerk',{[nan;nan;nan]},'psi',{[nan;nan;nan]});