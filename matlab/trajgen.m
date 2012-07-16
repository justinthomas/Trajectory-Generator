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
            minderiv = max(0,options{idx+1});
            if max(minderiv)>4; warning('This program can only support up to 4 derivatives at this time.'); end; %#ok<WNTAG>
        case 'ndim'
            % The number of dimensions we have.
            % For example, for a typical quadrotor, we have 4: x,y,z,psi
            d = options{idx+1}; 
            if d <= 0;  warning('Do you really want a <= 0 dimensional system?'); end %#ok<WNTAG>
    end
end

ticker = tic;
traj = [];
warning off MATLAB:nearlySingularMatrix

%% Keytimes, Segments

% How many segments do we have
N = size(waypoints,2) - 1;

keytimes = [waypoints.time]; % Keytimes

%% Generate our linear differential operator

D = differential_linear_operators(n);

%% Equality constraints

% Determine the size of E for preallocation.  There will be a row for every
% non-NaN and non-empty constraint.  This will also generate an error if 
% the dimensions of the waypoints are not consistent 
% (i.e. d is the not the same size for all waypoints)
nrows = sum(sum(~isnan([...
    [waypoints.pos],...
    [waypoints.vel],...
    [waypoints.acc],...
    [waypoints.jerk]])));

% Preallocate E and Ebeq
E = zeros(nrows,d*N*(n+1));
Ebeq = zeros(nrows,1);

% Initalize a row index for E and Ebeq
row = 1;

% And now populate E with the appropriate bases
for pt = 1:N+1
    % The matrix E will have 1 row for every constraint.
    % Additionally, it will have d*N*(n+1) columns.
    % Before each basis group (bgroup) in each row, there will be
    % ((idx - 1) + d*(seg-1))*(n+1) zeros where idx indexes the output (d)
    % and seg indexes the polynomial segment (N).  Then, we know the basis
    % will occupy (n+1) columns.  Finally, the resulting number of zeros is
    % simply the difference of the total columns in the row and the already
    % occupied columns.
    
    % Extract the time
    t = waypoints(pt).time;
    
    % We need to make the last waypoint fall on the end of the last
    % segment, not the beginning of the next segment.
    bgroup = min(pt,N);
    
    % Now, establish the constraints
    if ~isempty(waypoints(pt).pos)
        deriv = 0;
        temp = basis(t,deriv,n,D);
        for idx = 1:d
            if ~isnan(waypoints(pt).pos(idx)) && (deriv < minderiv(idx))
                startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
                E(row,startidx:startidx+n) = temp;
                Ebeq(row) = waypoints(pt).pos(idx);
                row = row+1;
            end
        end
    end
    
    if ~isempty(waypoints(pt).vel)
        deriv = 1;
        temp = basis(t,deriv,n,D);
        for idx = 1:d
            if ~isnan(waypoints(pt).vel(idx)) && (deriv < minderiv(idx))
                startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
                E(row,startidx:startidx+n) = temp;
                Ebeq(row) = waypoints(pt).vel(idx);
                row = row+1;
            end
        end
    end
    
    if ~isempty(waypoints(pt).acc)
        deriv = 2;
        temp = basis(t,deriv,n,D);
        for idx = 1:d
            if ~isnan(waypoints(pt).acc(idx)) && (deriv < minderiv(idx))
                startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
                E(row,startidx:startidx+n) = temp;
                Ebeq(row) = waypoints(pt).acc(idx);
                row = row+1;
            end
        end
    end
    
    if ~isempty(waypoints(pt).jerk)
        deriv = 3;
        temp = basis(t,deriv,n,D);
        for idx = 1:d
            if ~isnan(waypoints(pt).jerk(idx)) && (deriv < minderiv(idx))
                startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
                E(row,startidx:startidx+n) = temp;
                Ebeq(row) = waypoints(pt).jerk(idx);
                row = row+1;
            end
        end
    end
end

%% Continuity constraints

% There will be the same number of columns as in the matrix E, but now
% there will be a continunity constraint for each output that has a
% derivative below the one we are minimizing.

% There will be a continunity constraint (except for the first and last points)
% for each output and its derivatives below the one being minimized.
nrows = sum((N-1)*minderiv);
C = zeros(nrows,d*N*(n+1));

% This will be zeros since we have 
% basis' * coeffs1 - basis' * coeffs2 = 0
Cbeq = zeros(nrows,1);

% Initalize our row counter
row = 1;

% And now populate C with the appropriate bases
for pt = 2:N
    % The matrix C will have 1 row for every constraint.
    % Additionally, it will have d*N*(n+1) columns.
    % Before each basis group (bgroup) in each row, there will be
    % ((idx - 1) + d*(pt-1))*(n+1) zeros where idx indexes the output (d)
    % and pt indexes the polynomial segment (N).  Finally, we know the basis
    % will occupy (n+1) columns.
    
    % The first group will be one less than the point.  For example, for
    % the continunity constraints at waypoint 2, we will require
    % equivalency between groups 1 and 2.
    bgroup = pt - 1;
    
    % Extract the time at this particular waypoint
    t = waypoints(pt).time;
        
    % Loop through the derivatives
    for deriv = 0:max(minderiv)
        
    % Determine our basis at this timestep and for this derivative
    temp = basis(t,deriv,n,D);
    
        % Now loop through the dimensions
        for idx = 1:d
            
            % We don't want to impose constraints on derivatives higher
            % than or equal to what we are minimizing
            if deriv < minderiv(idx)
                
                % The first basis group starts here
                startidx = ((idx-1)+d*(bgroup-1))*(n+1)+1;
                C(row,startidx:startidx+n) = temp;
                
                % The second basis group starts (n+1)*d columns later
                startidx = startidx + (n+1)*d;
                C(row,startidx:startidx+n) = -temp; % Note the negative sign
                
                % Advance to the next row
                row = row+1;
            end
        end
    end
end

%% The H Matrix

% Initalize H
H = zeros((N+1)*d*(n+1));

% Determine the powers
powers = (n:-1:0)';

% Determine the Base Hpow matrix (this does not need to be recalculated for
% each flat output and segment)
Hpow_base = repmat(powers,1,n+1)+repmat(powers',n+1,1);

% Initalize our row indexes
rows = 1:(n+1);

% Loop through the segments
for seg = 1:N
    
    % Loop through the dimensions
    for idx = 1:d
        
        % Generate a matrix which represents the powers of H
        Hpow = Hpow_base-2*minderiv(idx);
        keyboard
        % Determine the coefficients
        if ~isequal(minderiv(idx),0)
            Hcoeffs = sum(D{minderiv(idx)})';
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
        
        % And store this block in H
        H(rows,rows) = Hcoeffs.*(keytimes(seg+1).^Hpow) - Hcoeffs.*(keytimes(seg).^Hpow);
        
        % Now increment to the next diagonal block
        rows = rows + (n+1);
    end
end

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
        
        %% Construct the problem
        
        problem.H = H;
        problem.Aeq = [E; C];
        problem.beq = [Ebeq; Cbeq];
        
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
        
    
    % Ensure that we do not get stuck in the trajectory generator
    if toc(ticker) > .3
        traj = [];
        warning('Trajectory Generator took too long'); %#ok<WNTAG>
        return
    end

end