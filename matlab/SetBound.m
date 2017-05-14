function bound = SetBound(interval,varargin)
% bound = SetBound(interval, varargin)
% 
% varargin:
%   specify a bound on 'pos', 'vel', 'acc', 'jerk', 'snap', or 'psi'
%   specify 'lb', 'ub'
%   specify the bound

%always require time
bound.time = interval;

% allowed inputs are:
allowed_in = {'pos','vel','acc','jerk','snap','psi'};
allowed_bound = {'lb', 'ub', '1norm', 'infnorm'};

for idx = 1:length(varargin)
    mask1 = strcmp(allowed_in, varargin(idx));    
    if any(mask1)
        mask2 = strcmp(allowed_bound, varargin(idx+1));
        if any(mask2);
            
            % Bound deriv is the numerical indication of the state which is
            % bounded.
            bound.state = allowed_in{mask1};
            bound.deriv = find(mask1) - 1;
            
            bound.type = allowed_bound{mask2};
            bound.arg = varargin{idx+2}(:);
        else
            warning('Argument %i does not have a valid bound label',idx);
        end
    end
end