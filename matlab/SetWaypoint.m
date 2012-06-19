function waypoint = SetWaypoint(time,varargin)

% allowed inputs are:
inputs = {'pos','vel','acc','jerk','psi'};

% this creates an empty struct with all the required inputs
%  note that the next two lines could be replaced with: for ii =
%  1:length(inputs);waypoint.(inputs{ii}) = [];end, but that would be
%  slower
temp = [inputs;cell(1,length(inputs))];
waypoint = struct('time',nan,temp{:});

%always require time
waypoint.time = time;


% loop through each input arg and check if it is a valid  input
for idx = 1:length(varargin)
    mask1 = strcmp(inputs, varargin(idx));
    if any(mask1);
        waypoint.(inputs{mask1}) =  varargin{idx+1}(:);
    end
end
