function output = ZeroWaypoint(time,ndim)

if nargin<1
    time = nan;
end

temp = zeros(ndim,1);
output = struct('time',{time},'pos',{temp},'vel',{temp},'acc',{temp},'jerk',{temp});