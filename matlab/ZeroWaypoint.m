function output = ZeroWaypoint(time, ndim)
% output = ZeroWaypoint(time, ndim)
%
% Generates a zero waypoint for use with trajgen.  The time of the waypoint
% is given by time and the dimension (e.g. # of x, y, z, ...) is given by
% ndim.

if nargin<1
    time = nan;
end

temp = zeros(ndim,1);
output = struct('time',{time},'pos',{temp},'vel',{temp},'acc',{temp},'jerk',{temp},'snap',{temp});

end