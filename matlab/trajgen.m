function traj = trajgen(waypoints, options, bounds, varargin)
% function traj = trajgen(waypoints, options, bounds, varargin)
%
% A more detailed explanation of this program will go here

%% Defaults

tstep = .001;                   % Default timestep to be used in 
numerical = false;            % Use quadprog
n = 10;      % Polynomial order

%% Process varargin
for idx = 1:2:length(options)
    switch options{idx}
        case 'polyorder'
            n = options{idx+1};
        case 'minderiv'
            % What derivative are we minimizing
            % minderiv = 4 corresponds to snap
            % minderiv = 2 corresponds to acceleration
            % minderiv = 0 corresponds to position
            minderiv = options{idx+1};
        case 'ndim'
            % The number of dimensions we have.
            % For example, for a regular quadrotor, we have 4: x,y,z,psi
            d = options{idx+1}; 
    end
end

ticker = tic;
traj = [];
warning off MATLAB:nearlySingularMatrix

%% Keyframes and times

% How many segments do we have
N = size(waypoints,2) - 1;

keytimes = [waypoints.time]; % Keytimes

%% Generate our linear differential operator

D = differential_linear_operators(n);

% Initalize the order and differential operators in the basis generators
basis(0, 0, n, D);

% coeffs has the coefficients along the rows, segments along columns, and
% flat output in the 3rd dimension.
coeffs = cell(d, N);
fo = cell(d,1);
D1fo = fo;
D2fo = fo;
D3fo = fo;
D4fo = fo;

% Determine the powers
powers = (n:-1:0)';

% Determine the Base Hpow matrix (this does not need to be recalculated for
% each flat output and segment)
Hpow_base = repmat(powers,1,n+1)+repmat(powers',n+1,1);

E = [];
for seg = 1:N
    % The matrix E will have 2 rows for every constraint except the first
    % and last waypoints.  Additionally, it will have d*N*(n+1) columns.
    % Before each basis group (in each row), there will be 
    % ((idx - 1) + d*(seg-1))*(n+1) zeros where idx indexes the output (d)
    % and seg indexes the polynomial segment (N).  Then, we know the basis
    % will occupy (n+1) columns.  Finally, the resulting number of zeros is
    % simply the difference of the total columns in the row and the already
    % occupied columns.
    
        if ~isempty(waypoints(seg).pos)
            t = waypoints(seg).time;
            for idx = 1:d
                E = [E;...
                    zeros(1,((idx-1)+d*(seg-1))*(n+1)), basis(t,0,n,D), zeros(1,(d*(N + 1 - seg) - idx)*(n+1))];
            end
        end
        
end
keyboard

optimize_count = 0;
while (1)   % Optimization loop
    optimize_count = optimize_count + 1;
    
    times = keytimes(seg):tstep:(keytimes(seg+1) - tstep);
    
    % Segment durations
    t = diff(keytimes);

    % Quadratic Optimization for each element for the current segment in our flat space
    for flat_out = 1:options.ndim
        
        % Determine H based on our two times.  Note: we have to add a
        % slight offset to ensure that H is positive semidefinite.
        
        if isequal(minderiv,4)
           numderiv = 4; 
            if isequal(flat_out, 4) % Yaw
                numderiv = 2;
            end
        elseif isequal(minderiv,0)
            numderiv = 0;
        end
        
        %% The H matrix

        % Generate a matrix which represents the powers of H
        Hpow = Hpow_base-2*numderiv;
        
        % Determine the coefficients
        if ~isequal(numderiv,0)
            Hcoeffs = D{numderiv}'*ones(n+1,1);
            Hcoeffs = Hcoeffs*Hcoeffs';
        else
            Hcoeffs = ones(n+1);
        end

        % Now integrate
        Hpow(Hpow >= 0) = Hpow(Hpow >= 0) + 1;
        Hcoeffs(Hpow > 0) = Hcoeffs(Hpow > 0)./Hpow(Hpow > 0);
        
        % Things with negative powers are actually zero.
        Hcoeffs(Hpow < 0) = 0;
        Hpow(Hpow < 0) = 0;
        
        %problem.H = Hcoeffs.*(t.^Hpow);
        problem.H = Hcoeffs.*(keytimes(seg+1).^Hpow) - Hcoeffs.*(keytimes(seg).^Hpow);

        %% Inequality constraints
        
        % I need to make sure z doesn't hit the ground.  Add it as a constraint
        % with A and b.
        
        % A*x <= b
        problem.Aineq = [];
        problem.bineq = [];
        
        %             % Establish our safe region constraints
        %             if flat_out < 4 && numerical
        %                 t = 0:5*tstep:t2-t1;
        %                 Aineq = -basisgen(t')';
        %                 bineq = -saferegion(flat_out)*ones(length(t),1);
        %
        %                 Aineq = [Aineq; basisgen(t')'];
        %                 bineq = [bineq; saferegion(flat_out+3)*ones(length(t),1)];
        %
        %                 problem.Aineq = Aineq;
        %                 problem.bineq = bineq;
        %
        %                 problem.options = optimset('Algorithm', 'active-set','MaxIter',1000);
        %
        %             end
        
        %% Equality constraints
        % Setup our boundary conditions based on the coefficients and
        % our basis
        
        % In the case where we are minimizing error, we are looking at a
        % receeding horizion problem and don't actually need to specify a
        % final boundary condition
        problem.Aeq = [...
            bcbasis(keytimes(seg),0, n, D);             % Initial position
            bcbasis(keytimes(seg),1, n, D);           % Initial velocity
            bcbasis(keytimes(seg),2, n, D);           % Initial Acceleration
            bcbasis(keytimes(seg),3, n, D)];           % Initial Jerk;
        
        % If the initial conditions are not set, leave them as free
        problem.Aeq = [];
        for idx = 1:floor(length(start)/4)
            problem.Aeq = [problem.Aeq;
                bcbasis(keytimes(seg),idx-1,n,D)];
        end
        
        if ~isempty(finish)
            problem.Aeq = [problem.Aeq;            
                bcbasis(keytimes(seg+1),0, n, D);       % Final Position
                bcbasis(keytimes(seg+1),1, n, D);       % Final velocity
                bcbasis(keytimes(seg+1),2, n, D);       % Final Acceleration
                bcbasis(keytimes(seg+1),3, n, D)];      % Final Jerk
        end
%         for idx = 1:size(constraints,2)
%             problem.Aeq = [problem.Aeq; bcbasis(0,
%         end
        
        % Here we actually specify the constraint
        problem.beq = [...
            s(flat_out,seg, 1);                                         %  Initial Position
            s(flat_out,seg, 2);                                    % Initial Velocity
            s(flat_out,seg, 3);                                  % Initial Acceleration
            s(flat_out,seg, 4)];                               % Jerk
        
        % Only set the initial conditions which are defined
        problem.beq = [];
        for deriv_p1 = 1:floor(length(start)/4)
            problem.beq = [problem.beq;
                s(flat_out, seg, deriv_p1)];
        end
        
        if ~isempty(finish)
            problem.beq = [problem.beq;
            s(flat_out,seg+1, 1);                                    % Final Position
            s(flat_out,seg+1, 2);                               % Final Velocity
            s(flat_out,seg+1, 3);                             % Final Acceleration
            s(flat_out,seg+1, 4)];                          % Jerk
        end
        %% Determine the solution
        
        % min x'*H*x  subject to:  A*x <= b and Aeq*x = beq
        if ~numerical
            
            temp = [2*problem.H, problem.Aeq';...
                problem.Aeq, zeros(size(problem.Aeq,1)')];
            
            % Analytic Solution
            out = temp\[zeros(n+1,1); problem.beq];
            if any(isnan(out)); keyboard; end;  %#### Remove this at some point!
            coeffs{flat_out,seg} = out(1:n+1);
            
        else
            % Set up the problem
            problem.options = optimset('MaxIter',1000,'Algorithm','interior-point-convex','Display','on');
            problem.solver = 'quadprog';
            
            % Numerical Solution
            coeffs{flat_out,seg} = quadprog(problem);
        end
        
    end
    
    % Ensure that we do not get stuck in the trajectory generator
    if toc(ticker) > .3
        traj = [];
        warning('Trajectory Generator took too long'); %#ok<WNTAG>
        return
    end
    
end

%% Save the trajectory

% This script collects everything and stores it in the traj struct
build_traj

end



% function vec = basisgen(t,order)
% % This function generates a basis vector of order n at times t.  It has a
% % dimension of n+1 by size(t,1) and takes the form [t.^n t.^(n-1) ... t 1]'
% 
% persistent n
% 
% if nargin > 1
%     n = order;
% end
% 
% if ~isequal(size(t,2),1)
%     error('t must be a column vector or scalar.');
% end
% 
% vec = t.^(n:-1:0)';
% 
% end
