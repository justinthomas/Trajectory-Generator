function bound = SetBound(interval,varargin)

%always require time
bound.time = interval;

% allowed inputs are:
allowed_in = {'pos','vel','acc','jerk','psi'};
allowed_bound = {'lb','ub','1norm', '2norm','infnorm'};

for idx = 1:length(varargin)
    mask1 = strcmp(allowed_in, varargin(idx));    
    if any(mask1)
        mask2 = strcmp(allowed_bound, varargin(idx+1));
        if any(mask2);
            bound.state = allowed_in{mask1};
            bound.type = allowed_bound{mask2};
            bound.arg = varargin{idx+2}(:);
        else
            warning('Argument %i does not have a valid bound label',idx);
        end
    end
end