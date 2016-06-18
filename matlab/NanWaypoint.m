function output = NanWaypoint(time, num_dimensions)
% waypoint = NanWaypoint(time, number_of_dimensions)

if nargin<1
    time = nan;
end

temp = NaN(num_dimensions,1);

output = struct('time',{time},'pos',{temp},'vel',{temp},'acc',{temp},'jerk',{temp},'snap',{temp});

end